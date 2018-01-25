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



	void RobotInit() override {
		leftTalon.SetName("left motor controller");
		rightTalon.SetName("right motor controller");
		drive.SetName("robot drive");
		navx.SetName("navx");
		pdp.SetName("power distribution panel");

	}




};

START_ROBOT_CLASS(Robot)
