__author__ = "seamonsters"

import wpilib
import ctre
from seamonsters.wpilib_sim import simulate
from seamonsters.modularRobot import Module
from seamonsters.gamepad import Gamepad
import seamonsters.gamepad
from seamonsters.logging import LogState

class Climber(Module):
    def lock(self):
        if not self.locked:
            self.locked = True
            self.climberMotor.changeControlMode(ctre.CANTalon.ControlMode.Position)
            self.climberMotor.setFeedbackDevice(ctre.CANTalon.FeedbackDevice.QuadEncoder)
            self.climberMotor.setPID(1.0, 0.0, 3.0, 0.0)
            self.lockPosition = self.climberMotor.getPosition()
        self.climberMotor.set(self.lockPosition)

    def unlock(self):
        if self.locked :
            self.locked = False
            self.climberMotor.changeControlMode(ctre.CANTalon.ControlMode.PercentVbus)

    def robotInit(self):
        self.gamepad = seamonsters.gamepad.globalGamepad(port = 0)

        self.climberMotor = ctre.CANTalon(4)

        self.lockLog = LogState("Climber lock mode")
        self.statusLog = LogState("Climber status")
        self.currentLog = LogState("Climber current")
        #self.encoderLog = LogState("Climber encoder")
        self.encoderLog = None

    def teleopInit(self):
        print("  Right trigger: Climb up")
        print("  Left trigger: Climb down")
        print("  Right bumper: Climb up slowly (lock mode only)")
        print("  Left bumper: Climb down slowly")
        print("  B: Lock motor")
        print("  X: Unlock motor")
        self.locked = False
        self.lockmode = False
        self.enabled = True
        self.lockPosition = None
        self.climberMotor.changeControlMode(ctre.CANTalon.ControlMode.PercentVbus)

    def teleopPeriodic(self):
        if self.gamepad.getRawButton(Gamepad.B):
            if not self.lockmode:
                self.lockmode = True
                #When the A button is pressed the motor locks, so the robot wont fall down the rope

        if self.gamepad.getRawButton(Gamepad.X):
            if self.lockmode:
                self.lockmode = False
            self.enabled = True
            #When the Right Joystick is pressed down the lockmode is disabled

        if not self.enabled:
            self.lockLog.update("Climber disabled!")
            self.statusLog.update("Locked, disabled!")
        elif self.lockmode:
            self.lockLog.update("On!")
        else:
            self.lockLog.update("Off")

        climbSpeed = self.gamepad.getRTrigger() - self.gamepad.getLTrigger()
        if abs(climbSpeed) < 0.05:
            climbSpeed = 0
        if climbSpeed == 0 and self.lockmode:
            self.lock()
            if self.enabled:
                self.statusLog.update("Locked!")
                if self.gamepad.getRawButton(Gamepad.RB):
                    self.lockPosition += 1500
                elif self.gamepad.getRawButton(Gamepad.LB):
                    self.lockPosition -= 1500
        elif self.enabled:
            self.unlock()
            self.climberMotor.set(climbSpeed)
            self.statusLog.update("Unlocked")

        self.currentLog.update(self.climberMotor.getOutputCurrent())
        #if self.climberMotor.getOutputCurrent() >= 200:
        #    self.lock()
        #    self.enabled = False

        if self.encoderLog != None:
            self.encoderLog.update(self.climberMotor.getPosition())

if __name__ == "__main__":
    wpilib.run(Climber)
