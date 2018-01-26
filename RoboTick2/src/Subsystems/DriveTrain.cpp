#include "DriveTrain.h"




DriveTrain::DriveTrain(WPI_TalonSRX& _leftTalon, WPI_TalonSRX& _rightTalon,
		DifferentialDrive& _drive, AHRS& _ahrs) : Subsystem("DriveTrain"),
		leftTalon{_leftTalon}, rightTalon{_rightTalon}, drive{_drive},ahrs{_ahrs}{

			AddChild(leftTalon);
			AddChild(rightTalon);
			AddChild(drive);
			AddChild(ahrs);


}

void DriveTrain::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	// SetDefaultCommand(new MySpecialCommand());
}


void DriveTrain::ArcadeDrive(double xSpeed, double zRotation) {
	drive.ArcadeDrive(xSpeed, zRotation, false);
}

void DriveTrain::CurvatureDrive(double xSpeed, double zRotation,
		bool isQuickTurn) {
	drive.CurvatureDrive(xSpeed, zRotation, isQuickTurn);
}

void DriveTrain::TankDrive(double leftSpeed, double rightSpeed) {
	drive.TankDrive(leftSpeed, rightSpeed, false);
}

void DriveTrain::stopMotors() {
	drive.StopMotor();
}


void DriveTrain::resetEncoders() {
	leftTalon.GetSensorCollection().SetQuadraturePosition(0, 0);
	rightTalon.GetSensorCollection().SetQuadraturePosition(0, 0);
}

void DriveTrain::resetGyro() {
	ahrs.Reset();
}
	// Put methods for controlling this subsystem
// here. Call these from Commands.
