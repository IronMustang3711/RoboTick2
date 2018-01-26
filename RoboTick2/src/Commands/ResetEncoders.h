/*
 * ResetEncoders.h
 *
 *  Created on: Jan 25, 2018
 *      Author: jason
 */

#ifndef SRC_COMMANDS_RESETENCODERS_H_
#define SRC_COMMANDS_RESETENCODERS_H_
#include <WPILib.h>
#include "../Subsystems/DriveTrain.h"
struct ResetEncoders : InstantCommand {

	DriveTrain& drive;

	ResetEncoders(DriveTrain& drive);

	void Execute() override;
};



struct ResetGyro: InstantCommand {
	DriveTrain& drive;

	ResetGyro(DriveTrain& drive);

	void Execute() override;
};


#endif /* SRC_COMMANDS_RESETENCODERS_H_ */
