/*
 * DriveForward.h
 *
 *  Created on: Jan 25, 2018
 *      Author: jason
 */

#pragma once

#include <WPILib.h>
//#include "../Subsystems/DriveTrain.h"
class DriveTrain;
struct DriveForward : Command {

	DriveForward(DriveTrain& drive);

	void Execute() override;
	bool IsFinished() override;

	DriveTrain& drive;
};


struct DriveForwardMotionMagic : Command {

	DriveForwardMotionMagic(DriveTrain& drive);

	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End() override;

	DriveTrain& drive;
};