from enum import Enum
import time as time
import math
# import RPi.GPIO as GPIO

class Direction(Enum):
    DIRECTION_CCW = 0
    DIRECTION_CW = 1

class MotorInterfaceType(Enum):
    FUNCTION = 0
    DRIVER = 1
    FULL2WIRE = 2
    FULL3WIRE = 3
    FULL4WIRE = 4
    HALF3WIRE = 6
    HALF4WIRE = 8

class IOType(Enum):
    OUTPUT = 1
    INPUT = 0

class IOValue(Enum):
    HIGH = 1
    LOW = 0
    
class AccelStepper:
    def __init__(self, interface:MotorInterfaceType, pin1:int, pin2:int, pin3:int=0, pin4:int=0, enable:bool=True, forward=None, backward=None):
        self._interface = interface
        self._currentPos = 0
        self._targetPos = 0
        self._speed = 0.0
        self._maxSpeed = 0.0
        self._acceleration = 0.0
        self._sqrt_twoa = 1.0
        self._stepInterval = 0
        self._minPulseWidth = 1
        self._enablePin = 0xff
        self._lastStepTime = 0
        self._pin = [pin1, pin2, pin3, pin4]
        self._enableInverted = False
        self._n = 0
        self._c0 = 0.0
        self._cn = 0.0
        self._cmin = 1.0
        self._direction = Direction.DIRECTION_CCW
        self._pinInverted = [0,0,0,0]
        self._forward = forward
        self._backward = backward
        if enable:
            self.enableOutputs()
        self.setAcceleration(1)
        self.setMaxSpeed(1)

    
    ''' New Function to Port from Arduino to RPI'''
    def micros(self):
        return time.perf_counter() * 1e6 # time us
    
    def delayMicroseconds(self,microseconds):
        start = time.monotonic()
        while True:
            elapsed = (time.monotonic() - start) * 1000000
            if elapsed >= microseconds:
                break
            
    def pinMode(self, pin:int, iotype:IOType):
        # GPIO.setmode(GPIO.BCM)
        # pi_iotype = GPIO.OUT if iotype == IOType.OUTPUT else GPIO.IN
        # GPIO.setup(pin, pi_iotype)
        pass

    def digitalWrite(self, pin:int, iovalue:IOValue):
        # pi_iovalue = GPIO.HIGH if iovalue == IOValue.HIGH else GPIO.LOW
        # GPIO.output(pin, pi_iovalue)
        pass
        
    ''' Converted functions from C++ to Python'''
    def moveTo(self, absolute):
        if self._targetPos != absolute:
            self._targetPos = absolute
            self.computeNewSpeed()

    def move(self, relative):
        self.moveTo(self._currentPos + relative)

    def runSpeed(self):
        if not self._stepInterval:
            return False
        time = self.micros()()
        if time - self._lastStepTime >= self._stepInterval:
            if self._direction == Direction.DIRECTION_CW:
                self._currentPos += 1
            else:
                self._currentPos -= 1
            self.step(self._currentPos)
            self._lastStepTime = time
            return True
        else:
            return False

    def distanceToGo(self):
        return self._targetPos - self._currentPos

    def targetPosition(self):
        return self._targetPos

    def currentPosition(self):
        return self._currentPos

    def setCurrentPosition(self, position):
        self._targetPos = self._currentPos = position
        self._n = 0
        self._stepInterval = 0
        self._speed = 0.0

    def computeNewSpeed(self):
        distanceTo = self.distanceToGo()
        stepsToStop = int((self._speed * self._speed) / (2.0 * self._acceleration))

        if distanceTo == 0 and stepsToStop <= 1:
            self._stepInterval = 0
            self._speed = 0.0
            self._n = 0
            return self._stepInterval

        if distanceTo > 0:
            if self._n > 0:
                if stepsToStop >= distanceTo or self._direction == Direction.DIRECTION_CCW:
                    self._n = -stepsToStop
            elif self._n < 0:
                if stepsToStop < distanceTo and self._direction == Direction.DIRECTION_CW:
                    self._n = -self._n
        elif distanceTo < 0:
            if self._n > 0:
                if stepsToStop >= -distanceTo or self._direction == Direction.DIRECTION_CW:
                    self._n = -stepsToStop
            elif self._n < 0:
                if stepsToStop < -distanceTo and self._direction == Direction.DIRECTION_CCW:
                    self._n = -self._n

        if self._n == 0:
            self._cn = self._c0
            self._direction = Direction.DIRECTION_CW if distanceTo > 0 else Direction.DIRECTION_CCW
        else:
            self._cn = self._cn - ((2.0 * self._cn) / ((4.0 * self._n) + 1))
            self._cn = max(self._cn, self._cmin)

        self._n += 1
        self._stepInterval = self._cn
        self._speed = 1000000.0 / self._cn
        if self._direction == Direction.DIRECTION_CCW:
            self._speed = -self._speed

        return self._stepInterval

    def run(self):
        if self.runSpeed():
            self.computeNewSpeed()
        return self._speed != 0.0 or self.distanceToGo() != 0

    def setMaxSpeed(self, speed):
        if speed < 0.0:
            speed = -speed
        if self._maxSpeed != speed:
            self._maxSpeed = speed
            self._cmin = 1000000.0 / speed
            if self._n > 0:
                self._n = int((self._speed * self._speed) / (2.0 * self._acceleration))
                self.computeNewSpeed()

    def maxSpeed(self):
        return self._maxSpeed

    def setAcceleration(self, acceleration):
        if acceleration == 0.0:
            return
        if acceleration < 0.0:
            acceleration = -acceleration
        if self._acceleration != acceleration:
            self._n = self._n * (self._acceleration / acceleration)
            self._c0 = 0.676 * math.sqrt(2.0 / acceleration) * 1000000.0
            self._acceleration = acceleration
            self.computeNewSpeed()

    def acceleration(self):
        return self._acceleration
    

    def setSpeed(self, speed):
        if speed == self._speed:
            return
        speed = min(max(speed, -self._maxSpeed), self._maxSpeed)
        if speed == 0.0:
            self._stepInterval = 0
        else:
            self._stepInterval = abs(1000000.0 / speed)
            self._direction = Direction.DIRECTION_CW if speed > 0.0 else Direction.DIRECTION_CCW
        self._speed = speed

    def speed(self):
        return self._speed

    def step(self, step):
        if self._interface == MotorInterfaceType.FUNCTION:
            self.step0(step)
        elif self._interface == MotorInterfaceType.DRIVER:
            self.step1(step)
        elif self._interface == MotorInterfaceType.FULL2WIRE:
            self.step2(step)
        elif self._interface == MotorInterfaceType.FULL3WIRE:
            self.step3(step)
        elif self._interface == MotorInterfaceType.FULL4WIRE:
            self.step4(step)
        elif self._interface == MotorInterfaceType.HALF3WIRE:
            self.step6(step)
        elif self._interface == MotorInterfaceType.HALF4WIRE:
            self.step8(step)

    def stepForward(self):
        self._currentPos += 1
        self.step(self._currentPos)
        self._lastStepTime = self.micros()()
        return self._currentPos

    def stepBackward(self):
        self._currentPos -= 1
        self.step(self._currentPos)
        self._lastStepTime = self.micros()()
        return self._currentPos
    







    def setOutputPins(self, mask):
        numpins = 2
        if self._interface == MotorInterfaceType.FULL4WIRE or self._interface == MotorInterfaceType.HALF4WIRE:
            numpins = 4
        elif self._interface == MotorInterfaceType.FULL3WIRE or self._interface == MotorInterfaceType.HALF3WIRE:
            numpins = 3
        for i in range(numpins):
            if mask & (1 << i):
                self.digitalWrite(self._pin[i], IOValue.HIGH ^ self._pinInverted[i])
            else:
                self.digitalWrite(self._pin[i], IOValue.LOW ^ self._pinInverted[i])
    def step0(self, step):
        if self._speed > 0:
            self._forward()
        else:
            self._backward()
    def step1(self, step):
        # Ignore the 'step' parameter
        self.setOutputPins(0b10 if self._direction else 0b00)
        self.setOutputPins(0b11 if self._direction else 0b01)
        self.delayMicroseconds(self._minPulseWidth)
        self.setOutputPins(0b10 if self._direction else 0b00)
    
    def step2(self, step):
        step &= 0x3
        switcher = {
            0: 0b10,
            1: 0b11,
            2: 0b01,
            3: 0b00
        }
        output_pins = switcher.get(step, 0)
        self.setOutputPins(output_pins)

    def step3(self, step):
        step %= 3
        switcher = {
            0: 0b100,
            1: 0b001,
            2: 0b010
        }
        output_pins = switcher.get(step, 0)
        self.setOutputPins(output_pins)

    def step4(self, step):
        step &= 0x3
        switcher = {
            0: 0b0101,
            1: 0b0110,
            2: 0b1010,
            3: 0b1001
        }
        output_pins = switcher.get(step, 0)
        self.setOutputPins(output_pins)

    
    def step6(self, step):
        switcher = {
            0: 0b100,
            1: 0b101,
            2: 0b001,
            3: 0b011,
            4: 0b010,
            5: 0b110
        }
        outputPins = switcher.get(step % 6, 0)
        self.setOutputPins(outputPins)

    def step8(self, step):
        switcher = {
            0: 0b0001,
            1: 0b0101,
            2: 0b0100,
            3: 0b0110,
            4: 0b0010,
            5: 0b1010,
            6: 0b1000,
            7: 0b1001
        }
        outputPins = switcher.get(step & 0x7, 0)
        self.setOutputPins(outputPins)

    
    def disableOutputs(self):
        if not self._interface:
            return
        self.setOutputPins(0)
        if self._enablePin != 0xff:
            self.pinMode(self._enablePin, IOType.OUTPUT)
            self.digitalWrite(self._enablePin, IOValue.LOW ^ self._enableInverted)

    def enableOutputs(self):
        if not self._interface:
            return
        self.pinMode(self._pin[0], IOType.OUTPUT)
        self.pinMode(self._pin[1], IOType.OUTPUT)
        if self._interface == MotorInterfaceType.FULL4WIRE or self._interface == MotorInterfaceType.HALF4WIRE:
            self.pinMode(self._pin[2], IOType.OUTPUT)
            self.pinMode(self._pin[3], IOType.OUTPUT)
        elif self._interface == MotorInterfaceType.FULL3WIRE or self._interface == MotorInterfaceType.HALF3WIRE:
            self.pinMode(self._pin[2], IOType.OUTPUT)
        if self._enablePin != 0xff:
            self.pinMode(self._enablePin, IOType.OUTPUT)
            self.digitalWrite(self._enablePin, IOValue.HIGH ^ self._enableInverted)
    
    def setMinPulseWidth(self, minWidth):
        self._minPulseWidth = minWidth

    def setEnablePin(self, enablePin):
        self._enablePin = enablePin
        if self._enablePin != 0xff:
            self.pinMode(self._enablePin, IOType.OUTPUT)
            self.digitalWrite(self._enablePin, IOValue.HIGH ^ self._enableInverted)

    def setPinsInverted(self, directionInvert, stepInvert, enableInvert):
        self._pinInverted[0] = stepInvert
        self._pinInverted[1] = directionInvert
        self._enableInverted = enableInvert

    def setPinsInverted(self, pin1Invert, pin2Invert, pin3Invert, pin4Invert, enableInvert):
        self._pinInverted[0] = pin1Invert
        self._pinInverted[1] = pin2Invert
        self._pinInverted[2] = pin3Invert
        self._pinInverted[3] = pin4Invert
        self._enableInverted = enableInvert

    def runToPosition(self):
        while self.run():
            yield

    def runSpeedToPosition(self):
        if self._targetPos == self._currentPos:
            return False
        if self._targetPos > self._currentPos:
            self._direction = Direction.DIRECTION_CCW
        else:
            self._direction = Direction.DIRECTION_CW
        return self.runSpeed()

    def runToNewPosition(self, position):
        self.moveTo(position)
        self.runToPosition()
    
    def stop(self):
        if self._speed != 0.0:
            steps_to_stop = int((self._speed * self._speed) / (2.0 * self._acceleration)) + 1 
            if self._speed > 0:
                self.move(steps_to_stop)
            else:
                self.move(-steps_to_stop)
    def is_running(self):
        return not (self._speed == 0.0 and self._targetPos == self._currentPos)