/*
 * CurvatureDrive.cpp
 *
 *  Created on: Jan 25, 2018
 *      Author: jason
 */

#include <Commands/CurvatureDrive.h>



CurvatureDrive::CurvatureDrive(DriveTrain& _drive, Joystick& _joystick) : Command("curvature drive"),
 drive{_drive}, joystick{_joystick}{

	 Requires(&drive);

}

void CurvatureDrive::Execute() {
	double y = -joystick.GetY();
	double z = -joystick.GetZ();

	z = z*z*z;
	y = y*y*y;

	drive.CurvatureDrive(y, z,joystick.GetRawButton(1));
}

bool CurvatureDrive::IsFinished() {
	return IsCanceled();
}
