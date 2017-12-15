import math
import wpilib
import ctre
import seamonsters

class ThisIsARobot(seamonsters.GeneratorBot):

    def robotInit(self):
        flRotate = ctre.CANTalon(10)
        flRotate.setFeedbackDevice(
            ctre.CANTalon.FeedbackDevice.QuadEncoder)
        flRotate.setPID(1.0, 0.0, 5.0, 0)

        frRotate = ctre.CANTalon(2)
        frRotate.setFeedbackDevice(
            ctre.CANTalon.FeedbackDevice.QuadEncoder)
        frRotate.setPID(1.0, 0.0, 5.0, 0)

        self.flDrive = ctre.CANTalon(1)
        self.frDrive = ctre.CANTalon(15)

        self.drive = seamonsters.SwerveDrive()
        self.drive.addWheel(1.0, -1.0, self.flDrive, flRotate, 3106)
        self.drive.addWheel(1.0, 1.0, self.frDrive, frRotate, 3106)

        self.joy = wpilib.Joystick(0)

    def teleop(self):
        print("initialize")
        self.drive.setDriveMode(seamonsters.DriveInterface.DriveMode.VOLTAGE)
        self.drive.zeroRotations()
        yield
        while True:
            magnitude = (self.joy.getMagnitude() ** 2) * .3
            direction = self.joy.getDirectionRadians()
            turn = 0
            if self.joy.getRawButton(4):
                turn = 0.2
            elif self.joy.getRawButton(5):
                turn = -0.2
            if magnitude < .1:
                magnitude = 0
                direction = 0
            self.drive.drive(magnitude, direction, turn)
            yield


if __name__ == "__main__":
    wpilib.run(ThisIsARobot, physics_enabled = True)
