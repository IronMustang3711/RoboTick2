#ifndef DriveTrain_H
#define DriveTrain_H

#include <WPILib.h>
#include <Commands/Subsystem.h>
#include <ctre/Phoenix.h>
#include <AHRS.h>

class DriveTrain : public Subsystem {
private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities
	WPI_TalonSRX& leftTalon;
	WPI_TalonSRX& rightTalon;
	DifferentialDrive& drive;
	AHRS& ahrs;


public:
	DriveTrain(WPI_TalonSRX& leftTalon, WPI_TalonSRX& rightTalon,DifferentialDrive& drive,AHRS& ahrs);
	void InitDefaultCommand();


	  void ArcadeDrive(double xSpeed, double zRotation);
	  void CurvatureDrive(double xSpeed, double zRotation, bool isQuickTurn);
	  void TankDrive(double leftSpeed, double rightSpeed);
	  void stopMotors();

};

#endif  // DriveTrain_H
