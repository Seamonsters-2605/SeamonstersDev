__author__ = "seamonsters"

import wpilib
import ctre
from seamonsters.modularRobot import Module
from seamonsters.gamepad import Gamepad
import seamonsters.gamepad

class Dance(Module):

    def robotInit(self):
        self.patterns = [
            [-1, 1, -1, 1], # up
            None,
            [1, 1, -1, -1], # right
            None
            [1, 1, 1, 1] # down
        ]

        self.gamepad = seamonsters.gamepad.globalGamepad(0)

        fl = ctre.CANTalon(2)
        fr = ctre.CANTalon(1)
        bl = ctre.CANTalon(0)
        br = ctre.CANTalon(3)
        self.talons = [fl, fr, bl, br]

        for talon in self.talons:
            talon.setControlMode(ctre.CANTalon.ControlMode.PercentVbus)

    def teleopInit(self):
        self.currentPattern = 0
        self.currentDirection = 0 # 0, 1, or -1

    def teleopPeriodic(self):

        if self.gamepad.buttonPressed(Gamepad.A):
            if self.currentDirection == 0:
                self.currentDirection = 1
            else:
                self.currentDirection *= -1
            self._randomColor()
        if self.gamepad.buttonPressed(Gamepad.X):
            self.currentDirection = 0

        dpad = self.gamepad.getDPad()
        if dpad >= 0 and dpad < len(self.patterns) \
                and self.patterns[dpad] is not None:
            self.currentPattern = dpad

        pattern = self.patterns[self.currentPattern]
        for i in range(0, 4):
            self.talons[i].set(pattern[i] * self.currentDirection)

        self.gamepad.updateButtons()

    def _randomColor(self):
        pass