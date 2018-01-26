/*
 * ArcadeDrive.cpp
 *
 *  Created on: Jan 25, 2018
 *      Author: jason
 */

#include <Commands/ArcadeDrive.h>

ArcadeDrive::ArcadeDrive(DriveTrain& _drive, Joystick& _joystick) : Command("arcade drive"),
		 drive{_drive}, joystick{_joystick}{
			 Requires(&drive);
		 }

void ArcadeDrive::Execute() {
	double y = -joystick.GetY();
	double z = -joystick.GetZ();

	z = z*z*z;
	y = y*y*y;

	drive.ArcadeDrive(y, z);
}



bool ArcadeDrive::IsFinished() {
	return IsCanceled();
}
