__author__ = "seamonsters"

import wpilib
import ctre
from seamonsters.wpilib_sim import simulate
from seamonsters.modularRobot import Module
from seamonsters.gamepad import Gamepad
import seamonsters.gamepad
from seamonsters.logging import LogState
from seamonsters import dashboard

class Shooter (Module):
    
    def robotInit(self):
        self.gamepad = seamonsters.gamepad.globalGamepad(port=0)

    def teleopInit(self):
        print("  Dpad up: Spin flywheel")
        print("  Dpad down: Reverse flywheel")
        print("  Dpad right: Feeder forwards")
        print("  Dpad left: Feeder backwards")

        if dashboard.getSwitch("Flywheel voltage mode", True):
            self.ballControl.getFlywheels().switchVoltageMode()
        else:
            self.ballControl.getFlywheels().switchSpeedMode()

        if dashboard.getSwitch("Fast shooting", False):
            print("Fast shooting!!")
            voltage = .9
        else:
            print("Slow shooting")
            voltage = .5
        self.ballControl.getFlywheels().speedVoltage = voltage
        self.ballControl.getFlywheels().speedSpeed = 27632 * voltage

    def teleopPeriodic(self):
        if self.gamepad.getRawButton(Gamepad.DOWN):
            self.ballControl.getFlywheels().reverseFlywheels()
        elif self.gamepad.getRawButton(Gamepad.UP) \
                or self.gamepad.getRawButton(Gamepad.RIGHT) \
                or self.gamepad.getRawButton(Gamepad.LEFT):
            self.ballControl.getFlywheels().spinFlywheels()
        else:
            self.ballControl.getFlywheels().stopFlywheels()

        if self.gamepad.getRawButton(Gamepad.RIGHT):
            self.ballControl.feed(1.0)
        elif self.gamepad.getRawButton(Gamepad.LEFT):
            self.ballControl.feed(-1.0)
        else:
            self.ballControl.feed(0.0)

    def setBallControl(self, ballControl):
        self.ballControl = ballControl

class BallControl:

    def __init__(self):
        self.feeder = ctre.CANTalon(7)
        self.flywheels = Flywheels()

    def getFlywheels(self):
        return self.flywheels

    def feed(self, speed):
        self.feeder.set(speed)

class Flywheels:

    def __init__(self):
        self.flywheelMotors = [ctre.CANTalon(5), ctre.CANTalon(6)]

        self.speedVoltage = .5

        self.speedSpeed = 27632 * self.speedVoltage

        # encoder resolution is 512 (* 4)
        for motor in self.flywheelMotors:
            motor.setPID(0.15, 0.0, 5.0, 0)
            motor.setFeedbackDevice(ctre.CANTalon.FeedbackDevice.QuadEncoder)

        for motor in self.flywheelMotors:
            motor.changeControlMode(ctre.CANTalon.ControlMode.PercentVbus)
        self.talonSpeedModeEnabled = False

        self.voltageModeStartupCount = 0

        self.speedLog = LogState("Flywheel speed")
        self.controlModeLog = LogState("Flywheel mode")

    def switchSpeedMode(self):
        self.speedModeEnabled = True

    def switchVoltageMode(self):
        self.speedModeEnabled = False

    def inSpeedMode(self):
        return self.speedModeEnabled

    def _talonSpeedMode(self):
        if not self.talonSpeedModeEnabled:
            self.talonSpeedModeEnabled = True
            for motor in self.flywheelMotors:
                motor.changeControlMode(ctre.CANTalon.ControlMode.Speed)

    def _talonVoltageMode(self):
        if self.talonSpeedModeEnabled:
            self.talonSpeedModeEnabled = False
            for motor in self.flywheelMotors:
                motor.changeControlMode(ctre.CANTalon.ControlMode.PercentVbus)

    def _updateLogs(self):
        self.speedLog.update(
            str(-self.flywheelMotors[0].getEncVelocity()) + ", "
            + str(-self.flywheelMotors[1].getEncVelocity()))
        if self.speedModeEnabled:
            self.controlModeLog.update("Speed")
        else:
            self.controlModeLog.update("Voltage!")

    def spinFlywheels(self):
        if self.speedModeEnabled:
            if self.voltageModeStartupCount < 50:
                self._talonVoltageMode()
                for motor in self.flywheelMotors:
                    motor.set(-self.speedVoltage)
            else:
                self._talonSpeedMode()
                for motor in self.flywheelMotors:
                    motor.set(-self.speedSpeed)
            self.voltageModeStartupCount += 1
        else:
            self._talonVoltageMode()
            for motor in self.flywheelMotors:
                motor.set(-self.speedVoltage)
        self._updateLogs()

    def stopFlywheels(self):
        self._talonVoltageMode()
        for motor in self.flywheelMotors:
            motor.set(0)
        self.voltageModeStartupCount = 0
        self._updateLogs()

    def reverseFlywheels(self):
        self._talonVoltageMode()
        for motor in self.flywheelMotors:
            motor.set(self.speedVoltage/2)
        self.voltageModeStartupCount = 0
        self._updateLogs()

if __name__ == "__main__":
    wpilib.run(Shooter)
