/*
 * ReadWaypoints.h
 *
 *  Created on: Jan 25, 2018
 *      Author: jason
 */

#ifndef SRC_COMMANDS_READWAYPOINTS_H_
#define SRC_COMMANDS_READWAYPOINTS_H_


#include <WPILib.h>
class ReadWaypoints : InstantCommand {
public:
	ReadWaypoints();
	void Execute() override;
};

#endif /* SRC_COMMANDS_READWAYPOINTS_H_ */
