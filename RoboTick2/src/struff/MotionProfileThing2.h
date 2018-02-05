#pragma once

#include <WPILib.h>
#include <ctre/Phoenix.h>
#include "../Subsystems/DriveTrain.h"
#include "ProfileArray.h"
#include <utility>
using namespace std;

class MotionProfileThing2: public Command {
public:

	static constexpr int SLOT = 0;
	static constexpr int TIMEOUT = 10;
	static constexpr int PROFILE_DURATION = 10; //ms
	static constexpr int encoder_ticks_per_roataion = 1410;

	static constexpr double NOTIFIER_PERIOD =( PROFILE_DURATION / 2.0) / 1000.0; // half of PROFILE_DURATION, in seconds


	WPI_TalonSRX& leftTalon;
	WPI_TalonSRX& rightTalon;

	Notifier notifier{&MotionProfileThing2::fastLoop,this}; //TODO: dont forget to start the notifier!

	MotionProfileThing2(WPI_TalonSRX& left, WPI_TalonSRX& right, DriveTrain& d) :
			Command("Motion Profile Thing 2"),
			leftTalon { left }, rightTalon {right } {
		Requires(&d);
	}

	void feedProfiles(WPI_TalonSRX& t){
		TrajectoryPoint point;

		for(auto& profData :TheProfiles){
			double position, velocity;//, duration;
			//nope std::tie(position,velocity,duration) = profData;

			 position = profData[0];
			 velocity = profData[1];
			// duration = profData[2];

			 point.position = position * encoder_ticks_per_roataion;
			 point.velocity =velocity * encoder_ticks_per_roataion / 600; //rpm -> encoder ticks/100ms
			 point.timeDur = TrajectoryDuration_10ms;
			 point.zeroPos = (&profData == &TheProfiles.front());
			 point.isLastPoint = (&profData == &TheProfiles.back());

			 t.PushMotionProfileTrajectory(point);


		}

	}


	void Initialize() override {
		for (auto t : { &leftTalon, &rightTalon }) {
			t->Config_kF(SLOT, 0.8, TIMEOUT);
			t->Config_kP(SLOT, 20.0, TIMEOUT);
			t->Config_kI(SLOT, 0.1, TIMEOUT);
			t->Config_kD(SLOT, 30.0, TIMEOUT);

			//t->ConfigMotionProfileTrajectoryPeriod(PROFILE_DURATION, TIMEOUT);
			t->ConfigMotionProfileTrajectoryPeriod(0, TIMEOUT); //duration already set in profile array

			//todo: what is this next call for?
			t->SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, PROFILE_DURATION, TIMEOUT);

			t->GetSensorCollection().SetQuadraturePosition(0, TIMEOUT);

			t->Set(ControlMode::MotionProfile,SetValueMotionProfile::Disable);

			t->ClearMotionProfileTrajectories();

			feedProfiles(*t);

		}


		notifier.StartPeriodic(NOTIFIER_PERIOD);


	}
	void Execute() override {
		fastLoop();




		for (auto t : { &leftTalon, &rightTalon }) {

			MotionProfileStatus status;

			t->GetMotionProfileStatus(status);

			if(status.btmBufferCnt > 5) {
				t->Set(ControlMode::MotionProfile,SetValueMotionProfile::Enable);
			} else if(status.activePointValid && status.isLast){
				t->Set(ControlMode::MotionProfile,SetValueMotionProfile::Hold);
				Cancel();
			}

			//TODO more diagnostic messages from status


			SmartDashboard::PutNumber("error: " + t->GetName(),
					t->GetClosedLoopError(SLOT));
			SmartDashboard::PutNumber("target: " + t->GetName(),
					t->GetClosedLoopTarget(SLOT));
			SmartDashboard::PutNumber("position: " + t->GetName(),
					t->GetSelectedSensorPosition(SLOT));
			SmartDashboard::PutNumber("velocity: " + t->GetName(),
					t->GetSelectedSensorVelocity(SLOT));

			SmartDashboard::PutNumber("trajectory position: "+t->GetName(),
					t->GetActiveTrajectoryPosition());
			SmartDashboard::PutNumber("trajectory velocity: "+t->GetName(),
					t->GetActiveTrajectoryVelocity());

		}

	}

	void fastLoop() {
		for (auto t : { &leftTalon, &rightTalon }) {
			t->ProcessMotionProfileBuffer();
		}
	}

	bool IsFinished() override {
		return false; //TODO!
	}

	void End() override {
		notifier.Stop();
	}

};
