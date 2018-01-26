/*
 * ReadWaypoints.cpp
 *
 *  Created on: Jan 25, 2018
 *      Author: jason
 */

#include "ReadWaypoints.h"
#include <support/json.h>
//#include <fstream>
#include <pathfinder.h>
#include <vector>
#include <support/raw_istream.h>
void from_json(const wpi::json& j, Waypoint& w ){
	w.x = j.at("x").get<double>();
	w.y = j.at("y").get<double>();
	w.angle = d2r(j.at("angle").get<double>());
}

ReadWaypoints::ReadWaypoints() : InstantCommand("read waypoints") {


}



void ReadWaypoints::Execute() {
	wpi::json json;
	{
		std::error_code errCode;
		wpi::raw_fd_istream ifs{"waypoints.json",errCode};
	//std::ifstream ifs{"waypoints.json"};
	ifs >> json;
	}

	auto waypoints_js = json["waypoints"];

	//todo: does this work?
	auto waypointss = json.at("waypoints").get<std::vector<Waypoint>>();


	std::vector<Waypoint> waypoints;

	waypoints.reserve(waypoints_js.size());

	for(auto& pt_js : waypoints_js){
		double x = pt_js["x"];
		double y = pt_js["y"];
		double heading = d2r(pt_js["heading"]);
		waypoints.push_back({x,y,heading});
		//waypoints.emplace_back(x,y,heading);
	}
}
