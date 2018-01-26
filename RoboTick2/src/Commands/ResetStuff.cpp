/*
 * ResetEncoders.cpp
 *
 *  Created on: Jan 25, 2018
 *      Author: jason
 */

#include "ResetStuff.h"

DrivetrainResetCommand::DrivetrainResetCommand(const llvm::Twine& name, DriveTrain& _drive)
: InstantCommand{name}, drive{_drive}
{
	Requires(&drive);
}

ResetEncoders::ResetEncoders(DriveTrain& _drive)
: DrivetrainResetCommand("reset encoders",_drive) { }

void ResetEncoders::Execute() {
	drive.resetEncoders();
}

ResetGyro::ResetGyro(DriveTrain& _drive)
: DrivetrainResetCommand("reset gyro", _drive){ }

void ResetGyro::Execute() {
	drive.resetGyro();
}


