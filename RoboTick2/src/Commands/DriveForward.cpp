/*
 * DriveForward.cpp
 *
 *  Created on: Jan 25, 2018
 *      Author: jason
 */

#include "DriveForward.h"
#include "../Subsystems/DriveTrain.h"


DriveForward::DriveForward(DriveTrain& _drive) : Command("drive forward"), drive{ _drive } {
}

void DriveForward::Execute() {
}

bool DriveForward::IsFinished() {
	return false;
}



DriveForwardMotionMagic::DriveForwardMotionMagic(DriveTrain & _drive) 
	: Command("drive forward(magic)"), drive{ _drive }
{
}

void DriveForwardMotionMagic::Initialize()
{
	drive.motionMagicInit();
}

void DriveForwardMotionMagic::Execute()
{
}

bool DriveForwardMotionMagic::IsFinished()
{
	return drive.motionMagicOnTarget();
}

void DriveForwardMotionMagic::End()
{
	drive.stopMotors();
}
