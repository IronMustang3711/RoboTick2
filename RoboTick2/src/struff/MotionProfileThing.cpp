#include "MotionProfileThing.h"
#include  <iterator>
//#include <sstream>
using namespace std;

MotionProfileThing::MotionProfileThing(WPI_TalonSRX & left_talon, WPI_TalonSRX & right_talon, DriveTrain& drive)
	: Command("Motion profile thing"), 
	updater(&MotionProfileThing::periodicTask, this),
	leftTalon{left_talon},
	rightTalon{right_talon}
{
		Requires(&drive);
}

void MotionProfileThing::generateAndLoadProfiles()
{
	//inches
	std::vector<Waypoint> waypoints = {
	{ 0   ,  0 , 0 },
	{ 72 ,  72, d2r(-90) },
	{ 144, 144, 0 }
	};


	TrajectoryCandidate ctx;

	pathfinder_prepare(
		waypoints.data(),
		waypoints.size(),
		FIT_HERMITE_CUBIC,
		PATHFINDER_SAMPLES_HIGH,
		TIME_STEP,
		MAX_V,
		MAX_A,
		MAX_JERK,
		&ctx);

	std::vector<Segment> base_traj{};
	std::vector<Segment> left_traj{};
	std::vector<Segment> right_traj{};
	base_traj.reserve(ctx.length);
	left_traj.reserve(ctx.length);
	right_traj.reserve(ctx.length);

	

	pathfinder_generate(&ctx, base_traj.data());

	pathfinder_modify_tank(
		base_traj.data(),
		ctx.length,
		left_traj.data(),
		right_traj.data(),
		WHEELBASE_WIDTH);


	TrajectoryPoint leftPoint;
	TrajectoryPoint rightPoint;

	for (size_t i = 0; i < base_traj.size(); ++i) {
		auto& left_seg = left_traj[i];
		auto& right_seg = right_traj[i];
		
		leftPoint.timeDur = rightPoint.timeDur = TrajectoryDuration_100ms;
		leftPoint.zeroPos = rightPoint.zeroPos = (i == 0);
		leftPoint.isLastPoint = rightPoint.isLastPoint = (i == base_traj.size() - 1);

		leftPoint.position = left_seg.position * encoder_ticks_per_inch;
		leftPoint.velocity = left_seg.velocity * encoder_ticks_per_inch;
		rightPoint.position = right_seg.position * encoder_ticks_per_inch;
		rightPoint.velocity = right_seg.velocity * encoder_ticks_per_inch;

		leftTalon.PushMotionProfileTrajectory(leftPoint);
		rightTalon.PushMotionProfileTrajectory(rightPoint);

	}

	SetTimeout(0.1 * base_traj.size() + 1.0);
}

void MotionProfileThing::periodicTask()
{
	leftTalon.ProcessMotionProfileBuffer();
	rightTalon.ProcessMotionProfileBuffer();
}



void MotionProfileThing::Initialize() {


	for (auto tr : { ref(leftTalon),ref(rightTalon) }) {
		WPI_TalonSRX& t = tr;
		t.Config_kF(SLOT, 0.842, TIMEOUT);
		t.Config_kP(SLOT, 20.0, TIMEOUT);
		t.Config_kI(SLOT, 0.0, TIMEOUT);
		t.Config_kD(SLOT, 0.0, TIMEOUT);

		t.ClearMotionProfileTrajectories();

		//t.get().ChangeMotionControlFramePeriod(50);
		t.ConfigMotionProfileTrajectoryPeriod(100, TIMEOUT);
		t.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 100, TIMEOUT);

		t.Set(ControlMode::MotionProfile, SetValueMotionProfile::Disable);

		t.SetSelectedSensorPosition(0,SLOT,TIMEOUT);

	}

	generateAndLoadProfiles();
	updater.StartPeriodic(0.05);


}

void MotionProfileThing::Execute() {
	//leftTalon.Set(ControlMode::MotionProfile,)
	periodicTask();

	for (auto& t : { ref(leftTalon),ref(rightTalon) }) {
		t.get().Set(ControlMode::MotionProfile, SetValueMotionProfile::Enable);
	}

	leftTalon.GetMotionProfileStatus(leftStatus);
	rightTalon.GetMotionProfileStatus(rightStatus);
	//
	//std::stringstream ss;

	//ss << "last: "<<leftStatus.isLast << '\n';

	//DriverStation::ReportWarning(ss.str());

	SmartDashboard::PutNumber("traj buf", leftTalon.GetMotionProfileTopLevelBufferCount());
	SmartDashboard::PutNumber("left err", leftTalon.GetClosedLoopError(SLOT));
	SmartDashboard::PutNumber("right err", rightTalon.GetClosedLoopError(SLOT));
	SmartDashboard::PutBoolean("traj valid", leftStatus.activePointValid);
	SmartDashboard::PutNumber("ll buf", leftStatus.btmBufferCnt);


}





bool MotionProfileThing::IsFinished() {
	return false;//IsTimedOut();
}

void MotionProfileThing::End() {
	updater.Stop();
	Command::End();
}
