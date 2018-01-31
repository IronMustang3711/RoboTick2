#include "DriveTrain.h"
#include "../Commands/TeleopDrives.h"
DriveTrain::DriveTrain(WPI_TalonSRX& _leftTalon, WPI_TalonSRX& _rightTalon,
		DifferentialDrive& _drive, AHRS& _ahrs) :
		Subsystem("DriveTrain"), leftTalon { _leftTalon }, rightTalon {
				_rightTalon }, drive { _drive }, ahrs { _ahrs } {

	AddChild(leftTalon);
	AddChild(rightTalon);
	AddChild(drive);
	AddChild(ahrs);

	leftTalon.ConfigVoltageCompSaturation(11.0, 10);
	leftTalon.EnableVoltageCompensation(true);
	rightTalon.ConfigVoltageCompSaturation(11.0, 10);
	rightTalon.EnableVoltageCompensation(true);

}



void DriveTrain::Drive(double outputMagnitude, double curve)
{
	double leftOutput, rightOutput;

	double m_sensitivity = 0.5;

	if (curve < 0) {
		double value = std::log(-curve);
		double ratio = (value - m_sensitivity) / (value + m_sensitivity);
		if (ratio == 0) ratio = .0000000001;
		leftOutput = outputMagnitude / ratio;
		rightOutput = outputMagnitude;
	}
	else if (curve > 0) {
		double value = std::log(curve);
		double ratio = (value - m_sensitivity) / (value + m_sensitivity);
		if (ratio == 0) ratio = .0000000001;
		leftOutput = outputMagnitude;
		rightOutput = outputMagnitude / ratio;
	}
	else {
		leftOutput = outputMagnitude;
		rightOutput = outputMagnitude;
	}
	//todo: clamp output to [-1,1];

	leftTalon.Set(ControlMode::PercentOutput, leftOutput);
	rightTalon.Set(ControlMode::PercentOutput, -rightOutput);
	//SetLeftRightMotorOutputs(leftOutput, rightOutput);
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

void DriveTrain::InitDefaultCommand() {
	Joystick* j = new Joystick(0);
	SetDefaultCommand(new teleop::ArcadeDrive(*this,*j));
	//SetDefaultCommand(new teleop::TeleopSelector(*this,*j));
}

void DriveTrain::resetGyro() {
	ahrs.Reset();
}

void DriveTrain::Periodic() {

	SmartDashboard::PutString("current command", GetCurrentCommandName());
}
