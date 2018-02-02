/*
 * TeleopDrives.cpp
 *
 *  Created on: Jan 25, 2018
 *      Author: jason
 */

#include "TeleopDrives.h"
#include "../Subsystems/DriveTrain.h"
namespace teleop {

AbstractTeleopDrive::AbstractTeleopDrive(const char* name,
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


teleop::TankDrive::TankDrive(DriveTrain& drive, Joystick& joystick)
: AbstractTeleopDrive { "tank drive", drive, joystick } { }


void teleop::TankDrive::Execute() {
	double l = -1*joystick.GetRawAxis(1);
	double r = -1*joystick.GetRawAxis(3);

	//l = l*l*l;
	//r = r*r*r;
	//SmartDashboard::PutNumber("tank input right: ",r);
	//SmartDashboard::PutNumber("tank input left: ",l);

	drive.TankDrive(l, r);
}

TeleopSelector::TeleopSelector(DriveTrain& drive, Joystick& joystick)
	: InstantCommand("teleop selector") {
	chooser = new SendableChooser<Command*>();

	chooser->SetName("teleop chooser");
	chooser->AddDefault("arcade drive", new ArcadeDrive(drive, joystick));
	chooser->AddObject("curvature drive", new CurvatureDrive(drive, joystick));
	chooser->AddObject("tank drive", new TankDrive(drive,joystick));
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


