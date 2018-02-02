#include "MotionProfileThing.h"
#include  <iterator>
MotionProfileThing::MotionProfileThing(WPI_TalonSRX & left_talon, WPI_TalonSRX & right_talon)
	: leftTalon{left_talon},rightTalon{right_talon},updater(&MotionProfileThing::periodicTask,this)
{
		initProfiles();
	
	updater.StartPeriodic(0.05);
}

void MotionProfileThing::initProfiles()
{

//inches
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
		MAX_V ,
		MAX_A ,
		MAX_JERK ,
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
	leftTalon.ChangeMotionControlFramePeriod(50);
	rightTalon.ChangeMotionControlFramePeriod(50);

	leftTalon.ClearMotionProfileTrajectories();
	rightTalon.ClearMotionProfileTrajectories();



	leftTalon.ConfigMotionProfileTrajectoryPeriod(0,10);
	rightTalon.ConfigMotionProfileTrajectoryPeriod(0,10);

	beginProfIndex = 0;

}

void MotionProfileThing::Execute() {
	leftTalon.GetMotionProfileStatus(leftStatus);
	rightTalon.GetMotionProfileStatus(rightStatus);
}

void MotionProfileThing::startFilling() {
	TrajectoryPoint leftPoint;
	TrajectoryPoint rightPoint;

	for (;beginProfIndex < profs.size()
					&& !leftTalon.IsMotionProfileTopLevelBufferFull()
					&& !rightTalon.IsMotionProfileTopLevelBufferFull();
			++beginProfIndex) {
		auto& prof = profs[beginProfIndex];
		leftPoint.timeDur = rightPoint.timeDur = TrajectoryDuration_100ms;
		leftPoint.zeroPos = rightPoint.zeroPos = (beginProfIndex == 0);
		leftPoint.isLastPoint = rightPoint.isLastPoint =  (beginProfIndex = profs.size() -1);

		//TODO: IMPORTANT: convert to encoder ticks!
		leftPoint.position = prof.left.position * encoder_ticks_per_inch;
		leftPoint.velocity = prof.left.velocity * encoder_ticks_per_inch;
		rightPoint.position = prof.right.position * encoder_ticks_per_inch;
		rightPoint.velocity = prof.right.velocity * encoder_ticks_per_inch;

		leftTalon.PushMotionProfileTrajectory(leftPoint);
		rightTalon.PushMotionProfileTrajectory(rightPoint);

	}


}
