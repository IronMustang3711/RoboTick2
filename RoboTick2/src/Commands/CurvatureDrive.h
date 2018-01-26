/*
 * CurvatureDrive.h
 *
 *  Created on: Jan 25, 2018
 *      Author: jason
 */

#ifndef SRC_COMMANDS_CURVATUREDRIVE_H_
#define SRC_COMMANDS_CURVATUREDRIVE_H_
#include <WPILib.h>
#include "../Subsystems/DriveTrain.h"

struct CurvatureDrive  : Command {

	CurvatureDrive(DriveTrain& drive,Joystick& joystick);

	void Execute() override;
	bool IsFinished() override;


	DriveTrain& drive;
	Joystick& joystick;
};

#endif /* SRC_COMMANDS_CURVATUREDRIVE_H_ */
