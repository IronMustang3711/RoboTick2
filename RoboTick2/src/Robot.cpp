/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include <string>

#include <WPILib.h>
#include <AHRS.h>
#include <ctre/Phoenix.h>
#include "Subsystems/DriveTrain.h"
#include "Commands/ArcadeDrive.h"
#include "Commands/CurvatureDrive.h"
#include "Commands/ResetEncoders.h"

using namespace frc;
using namespace std;
using namespace cs;
using namespace nt;


struct Robot : public TimedRobot {

	WPI_TalonSRX leftTalon{2};
	WPI_TalonSRX rightTalon{3};

	DifferentialDrive drive{leftTalon,rightTalon};

	AHRS navx{SPI::Port::kMXP};

	PowerDistributionPanel pdp{};

	DriveTrain drivetrainSubsystem{leftTalon,rightTalon,drive,navx};

	Joystick joystick{0};


	SendableChooser<Command*> teleopChooser{};


	Command* driveCommand;

	void RobotInit() override {
		leftTalon.SetName("left motor controller");
		rightTalon.SetName("right motor controller");
		drive.SetName("robot drive");
		navx.SetName("navx");
		pdp.SetName("power distribution panel");


		teleopChooser.AddDefault("arcade drive", new ArcadeDrive(drivetrainSubsystem,joystick));
		teleopChooser.AddObject("curvature drive", new CurvatureDrive(drivetrainSubsystem,joystick));
		SmartDashboard::PutData(new CurvatureDrive(drivetrainSubsystem,joystick));


		SmartDashboard::PutData("teleop modes",&teleopChooser);

		drive.SetSafetyEnabled(false);

		 leftTalon.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 0);
		 rightTalon.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 0);

		 rightTalon.SetSensorPhase(true);


		 SmartDashboard::PutData(new ResetEncoders(drivetrainSubsystem));
		 SmartDashboard::PutData(new ResetGyro(drivetrainSubsystem));


	}

	void TeleopPeriodic() override {
		Scheduler::GetInstance()->Run();
		SmartDashboard::PutData(&drivetrainSubsystem);

		SmartDashboard::PutNumber("left vel",leftTalon.GetSelectedSensorVelocity(0));
		SmartDashboard::PutNumber("right vel",rightTalon.GetSelectedSensorVelocity(0));

		SmartDashboard::PutNumber("left encoder", leftTalon.GetSelectedSensorPosition(0));
		SmartDashboard::PutNumber("right encoder", rightTalon.GetSelectedSensorPosition(0));



	}

	void TeleopInit() override {
		if(driveCommand != nullptr)
			driveCommand->Cancel();


		driveCommand = teleopChooser.GetSelected();
		if(driveCommand != nullptr)
			driveCommand->Start();

	}

	void DisabledPeriodic() override{
		if(driveCommand){
			driveCommand->Cancel();
			driveCommand = nullptr;
		}
	}




};

START_ROBOT_CLASS(Robot)
