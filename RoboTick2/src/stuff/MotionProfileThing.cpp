#include "MotionProfileThing.h"

MotionProfileThing::MotionProfileThing(WPI_TalonSRX & left_talon, WPI_TalonSRX & right_talon)
	: leftTalon{left_talon},rightTalon{right_talon},updater(&MotionProfileThing::periodicTask,this)
{
	leftTalon.ChangeMotionControlFramePeriod(50);
	rightTalon.ChangeMotionControlFramePeriod(50);
	
	updater.StartPeriodic(0.05);
}

void MotionProfileThing::initProfiles()
{
	//in inches:
	constexpr double TIME_STEP = 0.1; 
	constexpr double MAX_V = 9.0; // [in/sec]
	constexpr double MAX_A = MAX_V / 3.0; //[in/sec^2]
	constexpr double MAX_JERK = 40.0; //[[in/sec^2]/sec]] TODO: sum MAX_A from 0 to 1 by time_step ?
	constexpr double encoder_ticks_per_inch = 1.0; //115;
	constexpr double WHEELBASE_WIDTH = 19.5;

	std::vector<Waypoint> waypoints = {
	{ 0   ,  0 , 0 },
	{ -72 ,  72, -90 },
	{ -144, 144, 0 }
	};

	//for (auto& w : waypoints) {
	//	w.x *= encoder_ticks_per_inch;
	//	w.y *= encoder_ticks_per_inch;
	//	w.angle = d2r(w.angle);
	//}

	TrajectoryCandidate candidate;

	pathfinder_prepare(
		waypoints.data(),
		waypoints.size(),
		FIT_HERMITE_CUBIC,
		PATHFINDER_SAMPLES_HIGH,
		TIME_STEP,
		MAX_V * encoder_ticks_per_inch,
		MAX_A * encoder_ticks_per_inch,
		MAX_JERK * encoder_ticks_per_inch,
		&candidate);

	std::vector<Segment> base_traj{};
	std::vector<Segment> left_traj{};
	std::vector<Segment> right_traj{};
	base_traj.reserve(candidate.length);
	left_traj.reserve(candidate.length);
	right_traj.reserve(candidate.length);
	
	profs.clear();
	profs.reserve(candidate.length);

	pathfinder_generate(&candidate, base_traj.data());

	pathfinder_modify_tank(
		base_traj.data(),
		candidate.length,
		left_traj.data(),
		right_traj.data(),
		WHEELBASE_WIDTH);




	for (size_t i = 0; i < base_traj.size(); ++i) {
		auto& left_seg = left_traj[i];
		auto& right_seg = right_traj[i];
		profs.push_back({ { left_seg.position, left_seg.velocity},
						  { right_seg.position,right_seg.velocity} });
	}


}

void MotionProfileThing::periodicTask()
{
	leftTalon.ProcessMotionProfileBuffer();
	rightTalon.ProcessMotionProfileBuffer();
}

void MotionProfileThing::reset()
{
	leftTalon.ClearMotionProfileTrajectories();
	rightTalon.ClearMotionProfileTrajectories();
}
