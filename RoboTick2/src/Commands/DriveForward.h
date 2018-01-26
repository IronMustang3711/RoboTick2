/*
 * DriveForward.h
 *
 *  Created on: Jan 25, 2018
 *      Author: jason
 */

#ifndef SRC_COMMANDS_DRIVEFORWARD_H_
#define SRC_COMMANDS_DRIVEFORWARD_H_

#include <WPILib.h>
#include "../Subsystems/DriveTrain.h"

struct DriveForward : Command {

	DriveForward(DriveTrain& drive);

	void Execute() override;
	bool IsFinished() override;
};

#endif /* SRC_COMMANDS_DRIVEFORWARD_H_ */
