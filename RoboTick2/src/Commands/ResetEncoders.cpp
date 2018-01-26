/*
 * ResetEncoders.cpp
 *
 *  Created on: Jan 25, 2018
 *      Author: jason
 */

#include <Commands/ResetEncoders.h>



ResetEncoders::ResetEncoders(DriveTrain& _drive) : InstantCommand("reset encoders"), drive{_drive} {
	Requires(&drive);
}

void ResetEncoders::Execute() {
	drive.resetEncoders();
}

ResetGyro::ResetGyro(DriveTrain& _drive) : InstantCommand("reset gyro"), drive{_drive}{
	Requires(&drive);
}

void ResetGyro::Execute() {
	drive.resetGyro();
}
