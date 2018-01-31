#pragma once

#include <WPILib.h>
#include <Commands/Subsystem.h>
#include <ctre/Phoenix.h>
#include <AHRS.h>
namespace drivetrain_constants {
//raw sensor units / 100 ms;
constexpr int max_encoder_rate_per_100ms = 1350;




constexpr int encoder_ticks_per_rev = 1410;
constexpr double inches_per_encoder_tick = 0.00872;
constexpr double pi = 3.14159265358979323846;

namespace verify_measurements {
	constexpr double expected_wheel_radius = 2.0;

	constexpr double inches_per_rev = inches_per_encoder_tick * encoder_ticks_per_rev;
	constexpr double calculated_wheel_radius = inches_per_rev / (2.0*pi);
	constexpr double err = expected_wheel_radius - calculated_wheel_radius;
	constexpr double abs_err = err <= 0 ? -err : err;
	static_assert(abs_err < 0.05, "unexpected wheel radius!");

	constexpr int max_encoder_rate_per_s = max_encoder_rate_per_100ms * 10;
	constexpr double max_fps = (((1/12.0)*inches_per_encoder_tick )* max_encoder_rate_per_s) ; //9.8 fps. seems high?
	constexpr double max_mph = (max_fps / 5280.0) * 60*60; //6.7
	constexpr double max_rpm = (60 * max_encoder_rate_per_s) / double(encoder_ticks_per_rev); //575

}


constexpr double deg_2_rad(double angle_degrees) {
	return angle_degrees * pi / 180;
}
constexpr double inches_to_encoder_ticks(double inches) {
	return 1.0 / inches_per_encoder_tick * inches;
}
constexpr double encoder_ticks_to_inches(int ticks) {
	return inches_per_encoder_tick * ticks;
}





constexpr double FGain = 1023 / (max_encoder_rate_per_100ms - 0.1*max_encoder_rate_per_100ms); //0.84


}



class DriveTrain: public Subsystem {
private:

	WPI_TalonSRX& leftTalon;
	WPI_TalonSRX& rightTalon;
	DifferentialDrive& drive;
	AHRS& ahrs;

public:
	DriveTrain(WPI_TalonSRX& leftTalon, WPI_TalonSRX& rightTalon,
			DifferentialDrive& drive, AHRS& ahrs);
	void InitDefaultCommand();

	void Drive(double fwd, double rotate);
	void ArcadeDrive(double xSpeed, double zRotation);
	void CurvatureDrive(double xSpeed, double zRotation, bool isQuickTurn);
	void TankDrive(double leftSpeed, double rightSpeed);
	void stopMotors();

	void resetEncoders();
	void resetGyro();

	void Periodic() override;

	void profilingThing();

	void motionMagicInit();

	bool motionMagicOnTarget();


};

