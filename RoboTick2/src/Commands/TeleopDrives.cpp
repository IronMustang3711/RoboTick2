/*
 * TeleopDrives.cpp
 *
 *  Created on: Jan 25, 2018
 *      Author: jason
 */

#include "TeleopDrives.h"

namespace teleop {

AbstractTeleopDrive::AbstractTeleopDrive(const llvm::Twine& name,
		DriveTrain& _drive, Joystick& _joystick) :
		Command { name }, drive { _drive }, joystick { _joystick } {
	Requires(&drive);
}

bool AbstractTeleopDrive::IsFinished() {
	return IsCanceled();
}

ArcadeDrive::ArcadeDrive(DriveTrain& drive, Joystick& joystick) :
		AbstractTeleopDrive { "arcade drive", drive, joystick } { }

void ArcadeDrive::Execute() {
	double y = -joystick.GetY();
	double z = -joystick.GetZ();

	z = z * z * z;
	y = y * y * y;

	drive.ArcadeDrive(y, z);

}

CurvatureDrive::CurvatureDrive(DriveTrain& drive, Joystick& joystick) :
		AbstractTeleopDrive { "curvature drive", drive, joystick } { }

void CurvatureDrive::Execute() {
	double y = -joystick.GetY();
	double z = -joystick.GetZ();

	z = z * z * z;
	y = y * y * y;

	drive.CurvatureDrive(y, z, joystick.GetRawButton(1));
}

} // namespace teleop_drive
