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
        # -213,2795
        self.drive.addWheel(-1.0, 1.0, self.flDrive, flRotate, 3000)
        # -133,2966
        self.drive.addWheel(1.0, 1.0, self.frDrive, frRotate, 3000)

        self.joy = wpilib.Joystick(0)

    def teleop(self):
        print("initialize")
        self.drive.setDriveMode(seamonsters.DriveInterface.DriveMode.VOLTAGE)
        yield
        while True:
            magnitude = self.joy.getMagnitude() * .5
            direction = self.joy.getDirectionRadians()
            if magnitude < .1:
                self.drive.drive(0,0,0)
            else:
                self.drive.drive(magnitude, direction, 0)
            yield


if __name__ == "__main__":
    wpilib.run(ThisIsARobot, physics_enabled = True)
