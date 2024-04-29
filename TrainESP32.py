from pybricks.hubs import CityHub
from pybricks.pupdevices import DCMotor, Light, Remote
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from pybricks.iodevices import PUPDevice
from uerrno import ENODEV

hub = CityHub()

motor = DCMotor(Port.A)
#esp = PUPDevice(Port.B)

def deviceInfo(port):
    # Try to get the device, if it is attached.
    try:
        device = PUPDevice(port)
        print(port, device.info())
    except OSError as ex:
        if ex.args[0] == ENODEV:
            # No device found on this port.
            print(port, ": ---")
        else:
            raise

deviceInfo(Port.A)
deviceInfo(Port.B)

while False:
    wait(500)
    hub.light.on(Color.RED)
    wait(500)
    hub.light.on(Color.YELLOW)
    wait(500)
    hub.light.on(Color.GREEN)

    print(hub.battery.voltage())

speed = 0

direction = 0
"""
while True:
    wait(500)
    deviceInfo(Port.B)"""

class ButtonState:
    state = 0
    pstate = 0
    change = StopWatch()

buttonState = { 
    Button.RIGHT : ButtonState()
    ,Button.RIGHT_PLUS : ButtonState()
    ,Button.RIGHT_MINUS : ButtonState()
    ,Button.CENTER : ButtonState()
 } 

messageTimer = 0

fuse = 1

remote = Remote()

esp = PUPDevice(Port.B)

while True:
    wait(10)

    if messageTimer > 0:
        messageTimer -= 1

    pressed = remote.buttons.pressed()

    for b, s in buttonState.items():
        if s.state != s.pstate:
            s.change.reset()
        s.pstate = s.state
        s.state = 1 if b in pressed else 0

    if fuse == 1:
        if Button.LEFT_PLUS in pressed:
            speed += 1
        if Button.LEFT_MINUS in pressed:
            speed -= 1

    if Button.LEFT in pressed or fuse == 0:
        speed -= (speed > 0) - (speed < 0)

    if direction == 0:
        speed = 0

    if speed > 100:
        speed = 100
    if speed < -50:
        speed = -50

    motor.dc(speed * direction)

    s = buttonState[Button.CENTER]
    if (s.state == 0 and s.pstate == 1):
        if s.change.time() < 500 and abs(speed) < 30:
            if direction == 0:
                direction = 1
            else:
                direction = - direction
        else:
            direction = 0
        esp.write(0,[0x42 + direction])
        if s.change.time() > 2000:
            hub.system.shutdown()

    if (s.pstate == 1 and s.change.time() > 2000):
        remote.light.on(Color.RED)

    s = buttonState[Button.RIGHT]
    if (s.state == 0 and s.pstate == 1):
        esp.write(0,[0x46 if s.change.time() < 500 else 0x27])

    s = buttonState[Button.RIGHT_PLUS]
    if (s.state == 0 and s.pstate == 1):
        esp.write(0,[0x51])

    if (s.state == 1 and s.pstate == 0):
        esp.write(0,[0x52])

    s = buttonState[Button.RIGHT_MINUS]
    if (s.state == 0 and s.pstate == 1):
        esp.write(0,[0x53])

    if (s.state == 1 and s.pstate == 0):
        esp.write(0,[0x54])

"""
    s = buttonState[Button.RIGHT_MINUS]
    if (s.state == 0 and s.pstate == 1):
        esp = PUPDevice(Port.B)
        esp.write(0,[65])

    try:
        if messageTimer == 0 and speed == 0:
            if Button.RIGHT in pressed:
                direction = 0
                messageTimer = 10
                esp = PUPDevice(Port.B)
                esp.write(0,[66])
            if Button.RIGHT_PLUS in pressed:
                direction = 1
                messageTimer = 10
                esp = PUPDevice(Port.B)
                esp.write(0,[67])
            if Button.RIGHT_MINUS in pressed:
                direction = -1
                messageTimer = 10
                esp = PUPDevice(Port.B)
                esp.write(0,[65])
    except OSError as ex:
        print ("no remote :(")"""