/*
 * ReadWaypoints.cpp
 *
 *  Created on: Jan 25, 2018
 *      Author: jason
 */

#include <Commands/ReadWaypoints.h>
#include <support/json.h>
#include <fstream>
ReadWaypoints::ReadWaypoints() : InstantCommand("read waypoints") {


}



void ReadWaypoints::Execute() {
	wpi::json json;
	{
	std::ifstream ifs{"waypoints.json"};
	ifs >> json;
	}

	auto waypoints_js = json["waypoints"];

	assert(waypoints_js.size()>0);

	for(auto& pt_js : waypoints_js){
		double x = pt_js["x"];
		double y = pt_js["y"];
		double heading = pt_js["heading"];
	}
}
