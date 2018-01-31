/*
 * TeleopDrives.cpp
 *
 *  Created on: Jan 25, 2018
 *      Author: jason
 */

#include "TeleopDrives.h"
#include "../Subsystems/DriveTrain.h"
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
	SmartDashboard::PutString("Teleop","arcade");

	double y = -joystick.GetY();
	double z = -joystick.GetZ();

	z = z * z * z;
	y = y * y * y;

	drive.ArcadeDrive(y, z);

}

CurvatureDrive::CurvatureDrive(DriveTrain& drive, Joystick& joystick) :
		AbstractTeleopDrive { "curvature drive", drive, joystick } { }

void CurvatureDrive::Execute() {
	SmartDashboard::PutString("Teleop","Curvature");

	double y = -joystick.GetY();
	double z = -joystick.GetZ();

	z = z * z * z;
	y = y * y * y;

	drive.CurvatureDrive(y, z, joystick.GetRawButton(1));
}


TeleopSelector::TeleopSelector(DriveTrain& drive, Joystick& joystick)
	: InstantCommand("teleop selector") {
	chooser = new SendableChooser<Command*>();

	chooser->SetName("teleop chooser");
	chooser->AddDefault("arcade drive", new ArcadeDrive(drive, joystick));
	chooser->AddObject("curvature drive", new CurvatureDrive(drive, joystick));
	SmartDashboard::PutData(chooser);
	Requires(&drive);
}

void TeleopSelector::Execute()
{
	if (auto c = chooser->GetSelected()){
		std::string msg = "starting command: " + c->GetName();
		DriverStation::ReportError(msg);
		c->Start();
	}

}

 } // namespace teleop_drive
