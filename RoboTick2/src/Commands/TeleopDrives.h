/*
 * TeleopDrives.h
 *
 *  Created on: Jan 25, 2018
 *      Author: jason
 */

#pragma once

#include <WPILib.h>
#include "../Subsystems/DriveTrain.h"

namespace teleop {

struct AbstractTeleopDrive: Command {
	AbstractTeleopDrive(const llvm::Twine& name, DriveTrain& drive,
			Joystick& joystick);

	bool IsFinished() override;

	DriveTrain& drive;
	Joystick& joystick;
};

struct ArcadeDrive: AbstractTeleopDrive {
	ArcadeDrive(DriveTrain& drive, Joystick& joystick);

	void Execute() override;

};

struct CurvatureDrive: AbstractTeleopDrive {
	CurvatureDrive(DriveTrain& drive, Joystick& joystick);

	void Execute() override;

};

}
