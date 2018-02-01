/*
 * TeleopDrives.h
 *
 *  Created on: Jan 25, 2018
 *      Author: jason
 */

#pragma once


#pragma once
#include <WPILib.h>
#include "../Subsystems/DriveTrain.h"


class DriveTrain;

namespace teleop {




class AbstractTeleopDrive : public Command {
public:
	AbstractTeleopDrive(const char* name, DriveTrain& drive,
			Joystick& joystick);

protected:
	bool IsFinished() override;


	DriveTrain& drive;
	Joystick& joystick;
};

class ArcadeDrive: public AbstractTeleopDrive {
public:
	ArcadeDrive(DriveTrain& drive, Joystick& joystick);

protected:
	void Execute() override;

};

class CurvatureDrive:public AbstractTeleopDrive {
public:
	CurvatureDrive(DriveTrain& drive, Joystick& joystick);

protected:
	void Execute() override;

};
class TankDrive: public AbstractTeleopDrive {
public:
	TankDrive(DriveTrain& drive, Joystick& joystick);

protected:
	void Execute() override;

};

class TeleopSelector : public InstantCommand {
public:
	TeleopSelector(DriveTrain& drive, Joystick& joystick);

protected:
	void Execute() override;

private:
	SendableChooser<Command*>* chooser;
};

}
