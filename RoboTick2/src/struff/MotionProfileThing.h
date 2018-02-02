#pragma once 

#include <utility>
#include <vector>
#include <array>


#include <WPILib.h>
#include <ctre/Phoenix.h>
#include <pathfinder.h>
#include "../Subsystems/DriveTrain.h"

struct SimpleProf {
	double position;
	double velocity;
};
struct SimpleProfPair {
	SimpleProf left;
	SimpleProf right;
};

//in inches:
constexpr double TIME_STEP = 0.1; // 100ms
constexpr double MAX_V = 9.0; // [in/sec]
constexpr double MAX_A = MAX_V / 3.0; //[in/sec^2]
constexpr double MAX_JERK = 40.0; //[[in/sec^2]/sec]] TODO: sum MAX_A from 0 to 1 by time_step ?
constexpr double encoder_ticks_per_inch = 115;
constexpr double WHEELBASE_WIDTH = 19.5;
constexpr int SLOT = 0;
constexpr int TIMEOUT = 10;

struct MotionProfileThing :public Command {


	enum class State {
		Init = 0,
		Initialized,
		Uploading_Profiles,
		CheckStatus
	};

	State state = State::Init;

	MotionProfileStatus leftStatus;
	MotionProfileStatus rightStatus;



	MotionProfileThing(WPI_TalonSRX& left_talon, WPI_TalonSRX& right_talon,DriveTrain& drive);


	void generateAndLoadProfiles();

	void periodicTask();







	WPI_TalonSRX& leftTalon;
	WPI_TalonSRX& rightTalon;
	Notifier updater;


	SetValueMotionProfile mp_command = SetValueMotionProfile::Disable;

protected:
	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End() override;

};
