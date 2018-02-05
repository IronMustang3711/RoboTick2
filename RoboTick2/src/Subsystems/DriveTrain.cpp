#include "DriveTrain.h"
#include "../Commands/TeleopDrives.h"
#include <pathfinder.h>
#include <vector>
#include <functional>
using namespace std;
using namespace drivetrain_constants;



DriveTrain::DriveTrain(WPI_TalonSRX& _leftTalon, WPI_TalonSRX& _rightTalon,
		DifferentialDrive& _drive, AHRS& _ahrs) :
		Subsystem("DriveTrain"), leftTalon { _leftTalon }, rightTalon {
				_rightTalon }, drive { _drive }, ahrs { _ahrs } {


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
	drive.TankDrive(leftSpeed, -rightSpeed, false);
}

void DriveTrain::stopMotors() {
	drive.StopMotor();
}

void DriveTrain::resetEncoders() {
	leftTalon.SetSelectedSensorPosition(0, 0, 10);
	rightTalon.SetSelectedSensorPosition(0, 0, 10);


	leftTalon.GetSensorCollection().SetQuadraturePosition(0, 10);
	rightTalon.GetSensorCollection().SetQuadraturePosition(0, 10);
}

void DriveTrain::InitDefaultCommand() {
	Joystick* j = new Joystick(0);
	SetDefaultCommand(new teleop::TankDrive(*this,*j));
	//SetDefaultCommand(new teleop::TeleopSelector(*this,*j));
}

void DriveTrain::resetGyro() {
	ahrs.Reset();
}
constexpr int MOTION_MAGIC_SLOT = 0;
constexpr int TIMEOUT = 10;

void DriveTrain::Periodic() {
	Subsystem::Periodic();
//	SmartDashboard::PutData(this);
//	SmartDashboard::PutString("current command", GetCurrentCommandName());
//	SmartDashboard::PutNumber("left err", leftTalon.GetClosedLoopError(MOTION_MAGIC_SLOT));
//	SmartDashboard::PutNumber("right err", rightTalon.GetClosedLoopError(MOTION_MAGIC_SLOT));
//	SmartDashboard::PutNumber("left position",leftTalon.GetSelectedSensorPosition(MOTION_MAGIC_SLOT));
//	SmartDashboard::PutNumber("right position",rightTalon.GetSelectedSensorPosition(MOTION_MAGIC_SLOT));
//	SmartDashboard::PutNumber("left vel",leftTalon.GetSelectedSensorVelocity(MOTION_MAGIC_SLOT));
//	SmartDashboard::PutNumber("right vel",rightTalon.GetSelectedSensorVelocity(MOTION_MAGIC_SLOT));

//SmartDashboard::PutNumber("left output", leftTalon.Get());
//SmartDashboard::PutNumber("left position",leftTalon.GetSelectedSensorPosition(0));

//SmartDashboard::PutNumber("right output", rightTalon.Get());
//SmartDashboard::PutNumber("right position",rightTalon.GetSelectedSensorPosition(0));



}

void DriveTrain::profilingThing()
{


	//in inches:
	constexpr double TIME_STEP = 0.01;
	constexpr double MAX_V = 9.0; // [in/sec]
	constexpr double MAX_A = MAX_V / 3.0; //[in/sec^2]
	constexpr double MAX_JERK = 40.0; //[[in/sec^2]/sec]] TODO: sum MAX_A from (0 to 1) by time_step ?

	constexpr double WHEELBASE_WIDTH = 19.5;

	std::vector<Waypoint> waypoints = {
	{0   ,  0 , 0       },
	{-72 ,  72, -90    },
	{-144, 144, 0      }
	};

	for (auto& w : waypoints) {
		w.x *= encoder_ticks_per_inch;
		w.y *= encoder_ticks_per_inch;
		w.angle = d2r(w.angle);
	}

	TrajectoryCandidate candidate;

	pathfinder_prepare(
		waypoints.data(),
		waypoints.size(),
		FIT_HERMITE_CUBIC,
		PATHFINDER_SAMPLES_HIGH,
		TIME_STEP,
		MAX_V,
		MAX_A,
		MAX_JERK,
		&candidate);

	std::vector<Segment> base_traj{};
	std::vector<Segment> left_traj{};
	std::vector<Segment> right_traj{};
	base_traj.reserve(candidate.length);
	left_traj.reserve(candidate.length);
	right_traj.reserve(candidate.length);

	pathfinder_generate(&candidate, base_traj.data());

	pathfinder_modify_tank(
		base_traj.data(),
		candidate.length,
		left_traj.data(),
		right_traj.data(),
		WHEELBASE_WIDTH);


}


void DriveTrain::motionMagicInit()
{

	resetEncoders();



	auto config_talon = [](WPI_TalonSRX& talon)->void {

		talon.Config_kF(MOTION_MAGIC_SLOT, 1*FGain, TIMEOUT);
		talon.Config_kP(MOTION_MAGIC_SLOT, 5.0, TIMEOUT);
		talon.Config_kI(MOTION_MAGIC_SLOT, 0.0, TIMEOUT);
		talon.Config_kD(MOTION_MAGIC_SLOT, 300.0, TIMEOUT);

		constexpr double cruise_velocity = 600;//0.85 * FGain;

		//talon.ConfigMotionProfileTrajectoryPeriod(5, TIMEOUT);
		talon.ConfigMotionCruiseVelocity(cruise_velocity, TIMEOUT);
		talon.ConfigMotionAcceleration(0.5*cruise_velocity, TIMEOUT);
		//talon.Set(ControlMode::MotionMagic, inches_to_encoder_ticks(120.0));
	};
	config_talon(leftTalon);
	config_talon(rightTalon);

	leftTalon.SetNeutralMode(NeutralMode::Brake);
	rightTalon.SetNeutralMode(NeutralMode::Brake);

	constexpr double enc_ticks = inches_to_encoder_ticks(120.0);
	leftTalon.Set(ControlMode::MotionMagic,enc_ticks);
	rightTalon.Set(ControlMode::MotionMagic,enc_ticks);


//	for (auto talonRef : { ref(leftTalon),ref(rightTalon) }) {
//		auto& talon = talonRef.get();
//
//
//		talon.Config_kF(MOTION_MAGIC_SLOT, FGain, TIMEOUT);
//		talon.Config_kP(MOTION_MAGIC_SLOT, 0.0, TIMEOUT);
//		talon.Config_kI(MOTION_MAGIC_SLOT, 0.0, TIMEOUT);
//		talon.Config_kD(MOTION_MAGIC_SLOT, 0.0, TIMEOUT);
//
//		constexpr double cruise_velocity = 0.75 * FGain;
//
//		talon.ConfigMotionCruiseVelocity(cruise_velocity, TIMEOUT);
//		talon.ConfigMotionAcceleration(cruise_velocity, TIMEOUT);
//		talon.Set(ControlMode::MotionMagic, inches_to_encoder_ticks(120.0));
//
//
//	}


//	for (auto& talonRef : { ref(leftTalon),ref(rightTalon) }) {
//		talonRef.get().Set(ControlMode::MotionMagic, inches_to_encoder_ticks(120.0));
//	}

	//TODO: tune P,I,D
	//TODO: enable inductive breaking.


}



bool DriveTrain::motionMagicOnTarget()
{
	return leftTalon.GetClosedLoopError(1);
	return false; //TODO!
}
