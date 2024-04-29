#include <HardwareSerial.h>
#include <Adafruit_NeoPixel.h>

#include <I2S.h>
#include "FS.h"
#include "SPIFFS.h"

const int frequency = 440; // frequency of square wave in Hz
const int amplitude = 5000; // amplitude of square wave
const int sampleRate = 16000; // sample rate in Hz
const int bps = 16;

const int halfWavelength = (sampleRate / frequency); // half wavelength of square wave

int count = 0;

struct WAV_HEADER
{
  uint32_t ChunkID;
  uint32_t ChunkSize;
  uint32_t Format;
  uint32_t Subchunk1ID;
  uint32_t Subchunk1Size;
  uint16_t AudioFormat;
  uint16_t NumChannels;
  uint32_t SampleRate;
  uint32_t ByteRate;
  uint16_t BlockAlign;
  uint16_t BitsPerSample;
  uint32_t Subchunk2ID;
  //unsigned char stuff[40];
  uint32_t Subchunk2Size;
};

static_assert(sizeof(WAV_HEADER) == 44);

i2s_mode_t mode = I2S_PHILIPS_MODE; // I2S decoder is needed

class AudioClip
{
public:
  int16_t *audioData = nullptr;
  int numSamples = 0;
  int position = 0;

  int stagesOut[5] = {0,0,0,0,0};
  int stagesIn[5] = {0,0,0,0,0};

  int stage = 0;

 AudioClip(const char *fileName) : audioData(nullptr), numSamples(0), stagesOut({0,0,0,0,0}), stagesIn({0,0,0,0,0})
  {
    Serial.println(fileName);
    File file = SPIFFS.open(fileName);
    if (file)
    {
      Serial.println(file.available());
      WAV_HEADER wav;
      file.read(reinterpret_cast<uint8_t*>(&wav), sizeof(wav));
      Serial.println(wav.Subchunk2Size);
      Serial.println(wav.SampleRate);
      audioData = new int16_t[wav.Subchunk2Size];
      file.read(reinterpret_cast<uint8_t*>(audioData), wav.Subchunk2Size);
      numSamples = wav.Subchunk2Size / 2;      
      file.close();
    }
  }

  int16_t GetSample()
  {
    int16_t ret = 0;
    if (audioData)
    {
      ret = audioData[position];
      position++;
      if (position >= numSamples)
        position = 0;
    }
    return ret;
  }

  int IsFirst()
  {
    for (int i=0; i<5; i++)
      if (position == stagesOut[i])
        return i;
    return -1;
    //position == 0;
  }

  void Reset(int stage)
  {
    position = stagesIn[stage];
  }

  ~AudioClip()
  {
    if (audioData)
      delete[] audioData;
    audioData = nullptr;
  }
};

class AudioClipEx
{
public:
  AudioClip *start = nullptr;
  AudioClip *middle = nullptr;
  AudioClip *end = nullptr;

  AudioClip *current = nullptr;

  AudioClipEx(AudioClip *_start, AudioClip *_middle, AudioClip *_end) : start(_start), middle(_middle), end(_end), current(nullptr)
  {

  }

  void SetClip(AudioClip *_current, int stage)
  {
    current = _current;
    if (current)
      current->Reset(stage);
  }

  int16_t GetSample(bool play)
  {
    if (current == nullptr)
    {
      if (play)
        SetClip(start, 0);
    }
    else
    {
      if (int s = current->IsFirst() >= 0)
      {
        if (play)
        {
          if (current == end)
            SetClip(start, s);
          else
            SetClip(middle, s);
        }
        else
        {
          if (current == end)
            SetClip(nullptr, s);
          else
            SetClip(end, s);
        }
      }
    }
    if (current != nullptr)
      return current->GetSample();
    else
      return 0;
  }
};


void processCommand(uint8_t cmd);

class LPF2
{
public:

  static const unsigned char BYTE_NACK = 0x02;
  static const unsigned char BYTE_ACK = 0x04;
  static const unsigned char CMD_Type = 0x40;//   # set sensor type command
  static const unsigned char CMD_Select = 0x43;//   #sets modes on the fly
  static const unsigned char CMD_Mode = 0x49;//   # set mode type command
  static const unsigned char CMD_Baud = 0x52;//   # set the transmission baud rate
  static const unsigned char CMD_Vers = 0x5F;//   # set the version number
  static const unsigned char CMD_ModeInfo = 0x80;//  # name command
  static const unsigned char CMD_Data = 0xC0;//  # data command

  static const unsigned char CMD_LLL_SHIFT = 3;

  static const unsigned char NAME = 0x0;
  static const unsigned char RAW = 0x1;
  static const unsigned char Pct = 0x2;
  static const unsigned char SI = 0x3;
  static const unsigned char SYM = 0x4;
  static const unsigned char FCT = 0x5;
  static const unsigned char FMT = 0x80;

  static const unsigned char DATA8 = 0;
  static const unsigned char DATA16 = 1;
  static const unsigned char DATA32 = 2;
  static const unsigned char DATAF = 3;//  # Data type codes

  static const unsigned char ABSOLUTE = 16;
  static const unsigned char RELATIVE = 8;
  static const unsigned char DISCRETE = 4;

  static const unsigned char LENGTH_1 = 0x00;
  static const unsigned char LENGTH_2 = 0x08;
  static const unsigned char LENGTH_4 = 0x10;

  int txPin = 21;
  int rxPin = 20;
  HardwareSerial uart;
  unsigned char type = 66;

  bool connected = false;

  int acks = 0;

  unsigned char buff[24];

  class Command
  {
  public:
    static const int BUFF_SIZE = 24;
    unsigned char buff[BUFF_SIZE];
    int size;

    Command& Begin(unsigned char type)
    {
      memset(buff, 0, sizeof(buff));
      buff[0] = type;
      size = 1;
      return *this;
    }

    Command& Append32(uint32_t v)
    {
      memcpy(buff + size, &v, sizeof(v));
      size += sizeof(v);
      return *this;
    }

    Command& Append8(uint8_t v)
    {
      buff[size++] = v;
      return *this;
    }

    Command& BeginMode(char *s, int num)
    {
      memset(buff, 0, sizeof(buff));
      int len = strlen(s);
      buff[0] = CMD_ModeInfo | (len << 2) | num;
      buff[1] = NAME;
      size = 2;
      memcpy(buff + size, s, len);
      size += len;
      return *this;
    }

    void Send(HardwareSerial &uart)
    {
      unsigned char crc = 0;
      for (int i=0; i<size; i++)
      {
        crc ^= buff[i];
      }
      crc ^= 0xff;
      Append8(crc);
      //for (int i=0; i<size; i++)
      //  Serial.println(buff[i]);
      uart.write(buff, size);
      size = 0;
    }
  };

  Command cmd;

  LPF2(int _txPin, int _rxPin) : txPin(_txPin), rxPin(_rxPin), uart(1), type(66), connected(false), acks(0)
  {
    //uart
    pinMode(txPin, OUTPUT);
    pinMode(rxPin, INPUT);
  }

  void SetType()
  {
    //buff[0] = 
  }

  void Connect()
  {
    Serial.println("Connecting to hub! :3");
    connected = false;
    digitalWrite(txPin, 0);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    digitalWrite(txPin, 1);
    vTaskDelay(500 / portTICK_PERIOD_MS);

    uart.begin(2400, SERIAL_8N1, rxPin, txPin);
    uart.write(0);
    
    cmd.Begin(CMD_Type).Append8(type).Send(uart);
    cmd.Begin(CMD_Mode).Append8(0).Append8(0).Send(uart);
    cmd.Begin(CMD_Baud).Append32(115200).Send(uart);
    //cmd.Begin(CMD_Vers).Append32(2).Append32(2).Send(uart);
    cmd.BeginMode("int8", 0).Send(uart);
    cmd.Begin(CMD_ModeInfo | LENGTH_2 | 0).Append8(FCT).Append8(ABSOLUTE).Append8(1).Send(uart);                     
    cmd.Begin(CMD_ModeInfo | LENGTH_4 | 0).Append8(FMT).Append8(1).Append8(DATA8).Append8(3).Append8(0).Send(uart);
    vTaskDelay(5 / portTICK_PERIOD_MS);
    uart.write(0x04);

    for (int i=0; i<100; i++)
    {
      vTaskDelay(10 / portTICK_PERIOD_MS);
      int chr = uart.read();
      if (chr > 0)
        Serial.println(chr, HEX);
      if (chr == 0x04)
      {
        connected = true;
        break;
      }
    }

    if (connected)
    {
      Serial.println("Connected! :3");
      uart.begin(115200, SERIAL_8N1, rxPin, txPin);
    }
    else
      Serial.println("Fail...");
  }

  void CommandLoop()
  {
    if (connected)
    {
      for (int chr = uart.read(); chr >= 0; chr = uart.read())
      {
        
        if (chr != 0 && chr != BYTE_NACK)
          Serial.println(chr, HEX);
        if (chr == CMD_Select)
        {
          int mode = uart.read();
          int crc = uart.read();
        }
        else if (chr == 0x46)
        {
          int zero = uart.read();
          int b9 = uart.read();
          int whatever = uart.read();
          int cmd = uart.read();
          Serial.print("command! :3 ");
          Serial.println(cmd, HEX);
          processCommand(cmd);
          int crc = uart.read();
        }
        else if (chr == BYTE_NACK)
        {
          acks++;
          if (acks % 100 == 0)
          {
            Serial.print("ack: ");
            Serial.println(acks);
          }
          cmd.Begin(CMD_Data | LENGTH_1).Append8(0).Send(uart);
        }
      }
    }
    else
    {
      Connect();
    }
  }
};

LPF2 lpf2(21,20);

void setup() {
  Serial.begin(9600);
  delay(500);
  Serial.println("Starting :3");
  // put your setup code here, to run once:

  xTaskCreate(
    lights,    // Function that should be called
    "Lights",   // Name of the task (for debugging)
    1000,            // Stack size (bytes)
    NULL,            // Parameter to pass
    1,               // Task priority
    NULL             // Task handle
  );  

  xTaskCreate(
    audio,    // Function that should be called
    "Audio",   // Name of the task (for debugging)
    10000,            // Stack size (bytes)
    NULL,            // Parameter to pass
    1,               // Task priority
    NULL             // Task handle
  );  
}

void loop() {
  // put your main code here, to run repeatedly:
  //lpf2.connected = true;
  delay(10);
  lpf2.CommandLoop();
}

int dir = 0;

int strobe = 0;

int cabA = 0;
int cabB = 0;

int horn1 = 0;
int horn2 = 0;

void processCommand(uint8_t cmd)
{
  if (cmd >= 0x41 && cmd <= 0x43)
  {
    dir = cmd - 0x42;
  }

  switch (cmd)
  {
    case 0x44: strobe = 0; break;
    case 0x45: strobe = 1; break;
    case 0x46: strobe = 1 - strobe; break;
  
    case 0x21: cabA = 0; break;
    case 0x22: cabA = 1; break;
  
    case 0x23: cabB = 0; break;
    case 0x24: cabB = 1; break;

    case 0x25: cabA = 0; cabB = 0; break;
    case 0x26: cabA = 1; cabB = 1; break;

    case 0x27: cabA = 1 - cabA; cabB = cabA; break;

    case 0x51: horn1 = 0; break;
    case 0x52: horn1 = 1; break;
  
    case 0x53: horn2 = 0; break;
    case 0x54: horn2 = 1; break;
  }
}


void lights(void * parameter)
{

  Adafruit_NeoPixel ws(6, 1, NEO_RGB + NEO_KHZ800);

  ws.begin();

  ws.clear();

  //ws.setBrightness(100);

  ws.show();

  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);

  for(int i=0;;i++)
  { // infinite loop

    switch (dir)
    {
      case 0:
        ws.setPixelColor(0, ws.Color(0,0,0));
        ws.setPixelColor(1, ws.Color(0,0,0));
        ws.setPixelColor(4, ws.Color(0,0,0));
        ws.setPixelColor(5, ws.Color(0,0,0));

        digitalWrite(9, 0);
        digitalWrite(10, 0);
      break;
      case -1:
        ws.setPixelColor(0, strobe == 0 || i < 5 ? ws.Color(255,255,255) : ws.Color(0,0,0));
        ws.setPixelColor(1, strobe == 0 || i >= 5 ? ws.Color(255,255,255) : ws.Color(0,0,0));
        ws.setPixelColor(4, ws.Color(255,0,0));
        ws.setPixelColor(5, ws.Color(255,0,0));
        digitalWrite(9, 0);
        digitalWrite(10, 1);
      break;
      case 1:
        ws.setPixelColor(0, ws.Color(255,0,0));
        ws.setPixelColor(1, ws.Color(255,0,0));
        ws.setPixelColor(4, strobe == 0 || i < 5 ? ws.Color(255,255,255) : ws.Color(0,0,0));
        ws.setPixelColor(5, strobe == 0 || i >= 5 ? ws.Color(255,255,255) : ws.Color(0,0,0));
        digitalWrite(9, 1);
        digitalWrite(10, 0);
      break;
    }


    ws.setPixelColor(2, cabA ? ws.Color(255,100,0) : ws.Color(0,0,0));
    ws.setPixelColor(3, cabB ? ws.Color(255,100,0) : ws.Color(0,0,0));

    ws.show();
    
    delay(100);

    if (i >= 10)
      i = 0;
  }
}

void audio(void * parameter)
{

  if(!SPIFFS.begin(true)){
      Serial.println("SPIFFS Mount Failed");
      return;
  }

  AudioClipEx *horn1clip = new AudioClipEx(new AudioClip("/horn_1_1.wav"), new AudioClip("/horn_1_2.wav"), new AudioClip("/horn_1_3.wav"));

  horn1clip->start->stagesOut[0] = 1965;
  horn1clip->start->stagesOut[1] = 2201;
  horn1clip->start->stagesOut[2] = 4105;
  
  horn1clip->end->stagesIn[0] = 10610;
  horn1clip->end->stagesIn[1] = 7438;

  AudioClipEx *horn2clip = new AudioClipEx(new AudioClip("/horn_2_1.wav"), new AudioClip("/horn_2_2.wav"), new AudioClip("/horn_2_3.wav"));

  horn2clip->start->stagesOut[0] = 954;
  horn2clip->start->stagesOut[1] = 1420;
//  horn2clip->start->stages[1] = 1930;
  horn2clip->start->stagesOut[2] = 3198;
  horn2clip->start->stagesOut[3] = 5429;
  horn2clip->start->stagesOut[4] = 7549;
  horn2clip->end->stagesIn[0] = 8386;
  horn2clip->end->stagesIn[1] = 4899;

  // start I2S at the sample rate with 16-bits per sample
  if (!I2S.begin(mode, sampleRate, bps)) {
    Serial.println("Failed to initialize I2S!");
    while (1); // do nothing
  }
  Serial.println("I2S Initialized!");  

  int32_t sample = 0;

  for(int i=0;;i++)
  { // infinite loop

    int32_t s1 = horn1clip->GetSample(horn1);
    int32_t s2 = horn2clip->GetSample(horn2);

    sample = s1 + s2;

    if (sample < -32768)
      sample = -32768;

    if (sample > 32767)
      sample = 32767;

    if(mode == I2S_PHILIPS_MODE || mode == ADC_DAC_MODE){ // write the same sample twice, once for Right and once for Left channel
      I2S.write(sample); // Right channel
      I2S.write(sample); // Left channel
    }else if(mode == I2S_RIGHT_JUSTIFIED_MODE || mode == I2S_LEFT_JUSTIFIED_MODE){
      // write the same only once - it will be automatically copied to the other channel
      I2S.write(sample);
    }

  }
}