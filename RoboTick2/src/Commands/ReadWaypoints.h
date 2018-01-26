/*
 * ReadWaypoints.h
 *
 *  Created on: Jan 25, 2018
 *      Author: jason
 */
#pragma once

#include <WPILib.h>
class ReadWaypoints : InstantCommand {
public:
	ReadWaypoints();
	void Execute() override;
};


