#pragma once 

#include <utility>
#include <vector>
#include <array>


#include <WPILib.h>
#include <ctre/Phoenix.h>
#include <pathfinder.h>

struct SimpleProf {
	double position;
	double velocity;
};
struct SimpleProfPair {
	SimpleProf left;
	SimpleProf right;
};

struct MotionProfileThing {
	MotionProfileThing(WPI_TalonSRX& left_talon, WPI_TalonSRX& right_talon);


	void initProfiles();

	void periodicTask();

	void reset();

	std::vector<SimpleProfPair> profs;

	WPI_TalonSRX& leftTalon;
	WPI_TalonSRX& rightTalon;
	Notifier updater;


};
