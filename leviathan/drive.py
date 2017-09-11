__author__ = "seamonsters"

import wpilib
import ctre
from seamonsters.wpilib_sim import simulate
from seamonsters.modularRobot import Module
from seamonsters.drive import DriveInterface
from seamonsters.drive import AccelerationFilterDrive
from seamonsters.drive import FieldOrientedDrive
from seamonsters.drive import DynamicPIDDrive
from seamonsters.holonomicDrive import HolonomicDrive
from seamonsters.logging import LogState
from seamonsters import dashboard
import vision

from robotpy_ext.common_drivers.navx import AHRS
import math

class DriveBot(Module):

    def robotInit(self):
        ### CONSTANTS ###

        # if the joystick direction is within this number of radians on either
        # side of straight up, left, down, or right, it will be rounded
        self.driveDirectionDeadZone = math.radians(10)

        # rate of increase of velocity per 1/50th of a second:
        accelerationRate = 1.0

        # PIDF values for fast driving:
        fastPID = (1.0, 0.0009, 3.0, 0.0)
        # speed at which fast PID's should be used:
        fastPIDScale = 0.09
        # PIDF values for slow driving:
        slowPID = (10.0, 0.0009, 3.0, 0.0)
        # speed at which slow PID's should be used:
        slowPIDScale = 0.01

        pidLookBackRange = 10

        self.maxVelocity = 650

        self.joystickDeadzone = .1

        ### END OF CONSTANTS ###

        self.driverJoystick = wpilib.Joystick(0)

        fl = ctre.CANTalon(2)
        fr = ctre.CANTalon(1)
        bl = ctre.CANTalon(0)
        br = ctre.CANTalon(3)
        self.talons = [fl, fr, bl, br]

        for talon in self.talons:
            talon.setFeedbackDevice(ctre.CANTalon.FeedbackDevice.QuadEncoder)

        self.driveModeLog = LogState("Drive mode")

        self._setPID(fastPID)

        # encoder has 100 raw ticks -- with a QuadEncoder that makes 400 ticks
        # the motor gear has 12 teeth and the wheel has 85 teeth
        # 85 / 12 * 400 = 2833.333 = ~2833
        ticksPerWheelRotation = 2833
        self.holoDrive = HolonomicDrive(fl, fr, bl, br, ticksPerWheelRotation)
        self.holoDrive.invertDrive(True)
        self.holoDrive.setWheelOffset(math.radians(45.0))  # angle of rollers

        self.pidDrive = DynamicPIDDrive(self.holoDrive, self.talons,
                                        slowPID, slowPIDScale,
                                        fastPID, fastPIDScale, pidLookBackRange)

        self.filterDrive = AccelerationFilterDrive(self.pidDrive,
                                                   accelerationRate)

        self.ahrs = AHRS.create_spi()  # the NavX
        self.fieldDrive = FieldOrientedDrive(self.filterDrive, self.ahrs,
                                             offset=0)
        self.fieldDrive.zero()

        self.fieldDriveLog = LogState("Field oriented")

        self.pdp = wpilib.PowerDistributionPanel()
        self.currentLog = LogState("Drive current", logFrequency=2.0)

        self.encoderLog = LogState("Wheel encoders")
        self.speedLog = LogState("Wheel speeds")

        if self.pdp.getVoltage() < 12:
            print("Battery Level below 12 volts!!!")

        self.controlMode = "Strafe"

    def autonomousInit(self):
        self.vision = vision.Vision()
        self.turnAlignLog = LogState("Turn align")
        self.targetWidthLog = LogState("Target width")

        self.currentSpeed = 0

        if dashboard.getSwitch("Drive voltage mode", False):
            self.holoDrive.setDriveMode(DriveInterface.DriveMode.VOLTAGE)
        else:
            self.holoDrive.setDriveMode(DriveInterface.DriveMode.POSITION)

    def autonomousPeriodic(self):
        contours = self.vision.getContours()
        targetCenter = vision.Vision.targetCenter(contours)
        targetDimensions = vision.Vision.targetDimensions(contours)
        if targetCenter == None or targetDimensions == None:
            print("No vision!!")
            self.filterDrive.drive(0, 0, 0)
            return

        targetX = float(targetCenter[0]) / float(vision.Vision.WIDTH)
        centerDistance = targetX - vision.Vision.CENTER
        self.turnAlignLog.update("{0:.5f}".format(centerDistance))
        turnAmount = centerDistance * abs(centerDistance) * 0.7

        targetWidth = float(targetDimensions[0]) / float(vision.Vision.WIDTH)
        self.targetWidthLog.update("{0:.5f}".format(targetWidth))
        distance = 0.55 - targetWidth
        driveSpeed = distance * abs(distance) * 1.5

        if driveSpeed > 0.25:
            driveSpeed = 0.25
        if driveSpeed < -0.2:
            driveSpeed = -0.2

        self.currentSpeed += (driveSpeed - self.currentSpeed) / 3

        self.filterDrive.drive(driveSpeed, math.pi/2, -turnAmount)

    def teleopInit(self):
        print("DRIVE GAMEPAD:")
        print("  Left Joystick: Strafe/Drive")
        print("  Right Joystick: Turn")
        print("  Left Joystick Button: Slower")
        print("  Right Joystick Button: Faster")
        print("  A: Brake")
        print("  Y: Shake")
        print("  Start: Position Mode")
        print("  Back: Voltage mode")

        self.holoDrive.zeroEncoderTargets()
        self.holoDrive.setMaxVelocity(self.maxVelocity)
        self.count = 0

        self.teleopCommand = None

        if dashboard.getSwitch("Field oriented drive", False):
            self.drive = self.fieldDrive
        else:
            self.drive = self.filterDrive
        if dashboard.getSwitch("Drive voltage mode", False):
            self.holoDrive.setDriveMode(DriveInterface.DriveMode.VOLTAGE)
        else:
            self.holoDrive.setDriveMode(DriveInterface.DriveMode.POSITION)

        self.turnScale = 0.3
        self.joystickExponent = 2

        self.wheelsLocked = False

        self.encoderLoggingEnabled = dashboard.getSwitch("Encoder logging",
                                                         False)

    def teleopPeriodic(self):
        self.count = self.count + 1

        current = 0
        for talon in self.talons:
            current += talon.getOutputCurrent()
        if current > 50:
            self.currentLog.update(str(current) + "!")
        else:
            self.currentLog.update(current)
        if self.encoderLoggingEnabled:
            encoderLogText = ""
            for talon in self.talons:
                encoderLogText += str(talon.getPosition()) + " "
            self.encoderLog.update(encoderLogText)
            speedLogText = ""
            for talon in self.talons:
                speedLogText += str(talon.getEncVelocity()) + " "
            self.speedLog.update(speedLogText)

        # change drive mode with left side buttons
        if self.driverJoystick.getRawButton(6):
            self.drive.setDriveMode(DriveInterface.DriveMode.VOLTAGE)
        elif self.driverJoystick.getRawButton(7):
            self.drive.setDriveMode(DriveInterface.DriveMode.POSITION)
        self.driveModeLog.update(self._driveModeName(self.drive.getDriveMode()))

        # change control mode with right side buttons
        if self.driverJoystick.getRawButton(11):
            self.controlMode = "Tank"
        elif self.driverJoystick.getRawButton(10):
            self.controlMode = "Strafe"

        joystickX = self.driverJoystick.getX()
        joystickY = self.driverJoystick.getY()
        if abs(joystickX) < self.joystickDeadzone:
            joystickX = 0
        if abs(joystickY) < self.joystickDeadzone:
            joystickY = 0

        if self.driverJoystick.getRawButton(1) and \
                        (joystickX + joystickY) == 0:
            # lock wheels and don't allow driving
            if not self.wheelsLocked:
                print("Locking wheels")
                for talon in self.talons:
                    talon.enable()
                    talon.changeControlMode(ctre.CANTalon.ControlMode.Position)
                    talon.set(talon.getPosition())
                    talon.setPID(30.0, 0.0009, 3.0, 0.0)
                self.wheelsLocked = True
            return
        else:
            self.wheelsLocked = False

        if self.driverJoystick.getRawButton(11):
            print("Zero field oriented.")
            self.fieldDrive.zero()
        if self.drive is self.fieldDrive:
            self.fieldDriveLog.update("Enabled")
        else:
            self.fieldDriveLog.update("Disabled")

        scale = (1 - self.driverJoystick.getZ()) / 2
        turnScale = scale * self.turnScale
        exponent = self.joystickExponent

        if self.controlMode == "Tank":
            turn = self._joystickPower(-joystickX,
                                       exponent) * turnScale
            magnitude = -self._joystickPower(joystickY,
                                            exponent) * scale
            direction = math.pi / 2

            # I think this adds strafing onto existing joystick movt, but untested
            if self.driverJoystick.getRawButton(4):
                magnitude = ((magnitude ** 2) + (scale ** 2)) ** .5
                direction += math.atan(scale / magnitude)
            elif self.driverJoystick.getRawButton(5):
                magnitude = ((magnitude ** 2) + (scale ** 2)) ** .5
                direction -= math.atan(scale / magnitude)


        else: # self.controlMode == "Strafe", UNTESTED
            if self.driverJoystick.getRawButton(4):
                turn = -turnScale
            elif self.driverJoystick.getRawButton(5):
                turn = turnScale
            else:
                turn = 0

            magnitude = abs(joystickX) + abs(joystickY)
            if magnitude > 1:
                magnitude = 1
            magnitude = self._joystickPower(magnitude, exponent) * scale

            direction = self.driverJoystick.getDirectionRadians() # may need modification (+ pi/2)


        accelFilter = True

        if self.driverJoystick.getRawButton(8):
            magnitude = 1
            if self.count % 10 >= 5:
                direction = math.pi
            else:
                direction = 0
            accelFilter = False

        if accelFilter:
            self.drive.drive(magnitude, direction, turn)
        else:
            self.pidDrive.drive(magnitude, direction, turn)

    def _driveModeName(self, driveMode):
        if driveMode == DriveInterface.DriveMode.VOLTAGE:
            return "Voltage!"
        if driveMode == DriveInterface.DriveMode.SPEED:
            return "Speed!"
        if driveMode == DriveInterface.DriveMode.POSITION:
            return "Position"
        return "Unknown!"

    def _setPID(self, pid):
        for talon in self.talons:
            talon.setPID(pid[0], pid[1], pid[2], pid[3])

    def _joystickPower(self, value, exponent):
        newValue = float(abs(value)) ** float(exponent)
        if value < 0:
            newValue = -newValue
        return newValue

    def roundDirection(self, value, target):
        if abs(value - target) <= self.driveDirectionDeadZone:
            return target
        else:
            return value


if __name__ == "__main__":
    wpilib.run(DriveBot)

