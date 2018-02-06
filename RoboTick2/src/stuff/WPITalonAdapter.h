#pragma once
#include <SpeedController.h>
#include "ctre/phoenix/ErrorCode.h"
#include "ctre/phoenix/paramEnum.h"
#include "ctre/phoenix/MotorControl/ControlMode.h"
#include "ctre/phoenix/MotorControl/Faults.h"
#include "ctre/phoenix/MotorControl/StickyFaults.h"
#include "ctre/phoenix/MotorControl/FeedbackDevice.h"
#include "ctre/phoenix/MotorControl/StatusFrame.h"
#include "ctre/phoenix/MotorControl/NeutralMode.h"
#include "ctre/phoenix/MotorControl/LimitSwitchType.h"
#include "ctre/phoenix/MotorControl/RemoteSensorSource.h"
#include "ctre/phoenix/MotorControl/ControlFrame.h"
#include "ctre/phoenix/MotorControl/VelocityMeasPeriod.h"
#include "ctre/phoenix/MotorControl/SensorTerm.h"
#include "ctre/phoenix/Motion/TrajectoryPoint.h"
#include "ctre/phoenix/Motion/MotionProfileStatus.h"
#include "ctre/phoenix/MotorControl/SensorCollection.h"

namespace ctre {
namespace phoenix {
namespace motorcontrol {
namespace can {
class TalonSRX;
}
}
}
}

using namespace ctre::phoenix::motorcontrol;

class MyTalon: public frc::SpeedController {

	using TalonImpl = ctre::phoenix::motorcontrol::can::TalonSRX;
public:
	MyTalon(TalonImpl* impl);

	MyTalon(const MyTalon& other) = default;
	MyTalon(MyTalon&& other) = default;

	MyTalon& operator=(const MyTalon& other) = default;
	MyTalon& operator=(MyTalon&& other) = default;

	//PIDOutput
	inline void PIDWrite(double output) override {
		Set(output);
	}

	//SpeedController
	void Set(double speed) override;
	double Get() const override;
	void SetInverted(bool isInverted) override;
	bool GetInverted() const override;
	inline void Disable() override {
		StopMotor();
	}
	inline void StopMotor() override {
		Set(0);
	}

	//BaseMotorController (ctre)

	int GetDeviceID();
	void Set(ControlMode Mode, double value);
	void Set(ControlMode mode, double demand0, double demand1);
	void NeutralOutput();
	void SetNeutralMode(NeutralMode neutralMode);
	void EnableHeadingHold(bool enable);
	void SelectDemandType(bool value);
	//------ Invert behavior ----------//
	void SetSensorPhase(bool PhaseSensor);

	//----- general output shaping ------------------//
	ctre::phoenix::ErrorCode ConfigOpenloopRamp(double secondsFromNeutralToFull,
			int timeoutMs);
	ctre::phoenix::ErrorCode ConfigClosedloopRamp(
			double secondsFromNeutralToFull, int timeoutMs);
	ctre::phoenix::ErrorCode ConfigPeakOutputForward(double percentOut,
			int timeoutMs);
	ctre::phoenix::ErrorCode ConfigPeakOutputReverse(double percentOut,
			int timeoutMs);
	ctre::phoenix::ErrorCode ConfigNominalOutputForward(double percentOut,
			int timeoutMs);
	ctre::phoenix::ErrorCode ConfigNominalOutputReverse(double percentOut,
			int timeoutMs);
	ctre::phoenix::ErrorCode ConfigNeutralDeadband(double percentDeadband,
			int timeoutMs);
	//------ Voltage Compensation ----------//
	ctre::phoenix::ErrorCode ConfigVoltageCompSaturation(double voltage,
			int timeoutMs);
	ctre::phoenix::ErrorCode ConfigVoltageMeasurementFilter(
			int filterWindowSamples, int timeoutMs);
	void EnableVoltageCompensation(bool enable);
	//------ General Status ----------//
	double GetBusVoltage();
	double GetMotorOutputPercent();
	double GetMotorOutputVoltage();
	double GetOutputCurrent();
	double GetTemperature();
	//------ sensor selection ----------//
	ctre::phoenix::ErrorCode ConfigSelectedFeedbackSensor(
			RemoteFeedbackDevice feedbackDevice, int pidIdx, int timeoutMs);
	ctre::phoenix::ErrorCode ConfigSelectedFeedbackSensor(
			FeedbackDevice feedbackDevice, int pidIdx, int timeoutMs);
	ctre::phoenix::ErrorCode ConfigRemoteFeedbackFilter(int deviceID,
			RemoteSensorSource remoteSensorSource, int remoteOrdinal,
			int timeoutMs);
	ctre::phoenix::ErrorCode ConfigSensorTerm(SensorTerm sensorTerm,
			FeedbackDevice feedbackDevice, int timeoutMs);

	//------- sensor status --------- //
	int GetSelectedSensorPosition(int pidIdx);
	int GetSelectedSensorVelocity(int pidIdx);
	ctre::phoenix::ErrorCode SetSelectedSensorPosition(int sensorPos,
			int pidIdx, int timeoutMs);
	//------ status frame period changes ----------//
	ctre::phoenix::ErrorCode SetControlFramePeriod(ControlFrame frame,
			int periodMs);
	ctre::phoenix::ErrorCode SetStatusFramePeriod(StatusFrame frame,
			int periodMs, int timeoutMs);
	ctre::phoenix::ErrorCode SetStatusFramePeriod(StatusFrameEnhanced frame,
			int periodMs, int timeoutMs);
	int GetStatusFramePeriod(StatusFrame frame, int timeoutMs);
	int GetStatusFramePeriod(StatusFrameEnhanced frame, int timeoutMs);
	//----- velocity signal conditionaing ------//
	ctre::phoenix::ErrorCode ConfigVelocityMeasurementPeriod(
			VelocityMeasPeriod period, int timeoutMs);
	ctre::phoenix::ErrorCode ConfigVelocityMeasurementWindow(int windowSize,
			int timeoutMs);
	//------ remote limit switch ----------//
	 ctre::phoenix::ErrorCode ConfigForwardLimitSwitchSource(
		RemoteLimitSwitchSource type, LimitSwitchNormal normalOpenOrClose,
		int deviceID, int timeoutMs);
	 ctre::phoenix::ErrorCode ConfigReverseLimitSwitchSource(
		RemoteLimitSwitchSource type, LimitSwitchNormal normalOpenOrClose,
		int deviceID, int timeoutMs);
	void OverrideLimitSwitchesEnable(bool enable);
	////------ local limit switch ----------//
	 ctre::phoenix::ErrorCode ConfigForwardLimitSwitchSource(LimitSwitchSource type,
		LimitSwitchNormal normalOpenOrClose, int timeoutMs);
	 ctre::phoenix::ErrorCode ConfigReverseLimitSwitchSource(LimitSwitchSource type,
		LimitSwitchNormal normalOpenOrClose, int timeoutMs);
	////------ soft limit ----------//
	 ctre::phoenix::ErrorCode ConfigForwardSoftLimitThreshold(int forwardSensorLimit,
		int timeoutMs);
	 ctre::phoenix::ErrorCode ConfigReverseSoftLimitThreshold(int reverseSensorLimit,
		int timeoutMs);
	 ctre::phoenix::ErrorCode ConfigForwardSoftLimitEnable(bool enable,
		int timeoutMs);
	 ctre::phoenix::ErrorCode ConfigReverseSoftLimitEnable(bool enable,
		int timeoutMs);
	 void OverrideSoftLimitsEnable(bool enable);
	//------ Current Lim ----------//
	/* not available in base */
	//------ General Close loop ----------//
	ctre::phoenix::ErrorCode Config_kP(int slotIdx, double value,
			int timeoutMs);
	ctre::phoenix::ErrorCode Config_kI(int slotIdx, double value,
			int timeoutMs);
	ctre::phoenix::ErrorCode Config_kD(int slotIdx, double value,
			int timeoutMs);
	ctre::phoenix::ErrorCode Config_kF(int slotIdx, double value,
			int timeoutMs);
	ctre::phoenix::ErrorCode Config_IntegralZone(int slotIdx, int izone,
			int timeoutMs);
	ctre::phoenix::ErrorCode ConfigAllowableClosedloopError(int slotIdx,
			int allowableCloseLoopError, int timeoutMs);
	ctre::phoenix::ErrorCode ConfigMaxIntegralAccumulator(int slotIdx,
			double iaccum, int timeoutMs);
	//------ Close loop State ----------//
	ctre::phoenix::ErrorCode SetIntegralAccumulator(double iaccum, int pidIdx,
			int timeoutMs);
	int GetClosedLoopError(int pidIdx);
	double GetIntegralAccumulator(int pidIdx);
	double GetErrorDerivative(int pidIdx);

	ctre::phoenix::ErrorCode SelectProfileSlot(int slotIdx, int pidIdx);

	int GetClosedLoopTarget(int pidIdx);
	int GetActiveTrajectoryPosition();
	int GetActiveTrajectoryVelocity();
	double GetActiveTrajectoryHeading();

	//------ Motion Profile Settings used in Motion Magic  ----------//
	ctre::phoenix::ErrorCode ConfigMotionCruiseVelocity(int sensorUnitsPer100ms,
			int timeoutMs);
	ctre::phoenix::ErrorCode ConfigMotionAcceleration(
			int sensorUnitsPer100msPerSec, int timeoutMs);
	//------ Motion Profile Buffer ----------//
	void ClearMotionProfileTrajectories();
	int GetMotionProfileTopLevelBufferCount();
	ctre::phoenix::ErrorCode PushMotionProfileTrajectory(
			const ctre::phoenix::motion::TrajectoryPoint & trajPt);
	bool IsMotionProfileTopLevelBufferFull();
	void ProcessMotionProfileBuffer();
	ctre::phoenix::ErrorCode GetMotionProfileStatus(
			ctre::phoenix::motion::MotionProfileStatus & statusToFill);
	ctre::phoenix::ErrorCode ClearMotionProfileHasUnderrun(int timeoutMs);
	ctre::phoenix::ErrorCode ChangeMotionControlFramePeriod(int periodMs);
	ctre::phoenix::ErrorCode ConfigMotionProfileTrajectoryPeriod(
			int baseTrajDurationMs, int timeoutMs);
	//------ error ----------//
	ctre::phoenix::ErrorCode GetLastError();
	//------ Faults ----------//
	ctre::phoenix::ErrorCode GetFaults(Faults & toFill);
	ctre::phoenix::ErrorCode GetStickyFaults(StickyFaults & toFill);
	ctre::phoenix::ErrorCode ClearStickyFaults(int timeoutMs);
	//------ Firmware ----------//
	int GetFirmwareVersion();
	bool HasResetOccurred();
	//------ Custom Persistent Params ----------//
	ctre::phoenix::ErrorCode ConfigSetCustomParam(int newValue, int paramIndex,
			int timeoutMs);
	int ConfigGetCustomParam(int paramIndex, int timeoutMs);
	//------ Generic Param API, typically not used ----------//
	ctre::phoenix::ErrorCode ConfigSetParameter(ctre::phoenix::ParamEnum param,
			double value, uint8_t subValue, int ordinal, int timeoutMs);
	double ConfigGetParameter(ctre::phoenix::ParamEnum param, int ordinal,
			int timeoutMs);
	//------ Misc. ----------//
	int GetBaseID();
	ControlMode GetControlMode();
	// ----- Follower ------//
	// void Follow(IMotorController & masterToFollow); //TODO!!
	void ValueUpdated();

	//------ RAW Sensor API ----------//
	/**
	 * @retrieve object that can get/set individual RAW sensor values.
	 */
	ctre::phoenix::motorcontrol::SensorCollection & GetSensorCollection();

	ctre::phoenix::ErrorCode ConfigSelectedFeedbackSensor(
			FeedbackDevice feedbackDevice, int pidIdx, int timeoutMs);
	ctre::phoenix::ErrorCode ConfigSelectedFeedbackSensor(
			RemoteFeedbackDevice feedbackDevice, int pidIdx, int timeoutMs);

	ctre::phoenix::ErrorCode SetStatusFramePeriod(StatusFrameEnhanced frame,
			int periodMs, int timeoutMs);
	ctre::phoenix::ErrorCode SetStatusFramePeriod(StatusFrame frame,
			int periodMs, int timeoutMs);

	int GetStatusFramePeriod(StatusFrameEnhanced frame, int timeoutMs);
	int GetStatusFramePeriod(StatusFrame frame, int timeoutMs);

	//------ Velocity measurement ----------//
	ctre::phoenix::ErrorCode ConfigVelocityMeasurementPeriod(
			VelocityMeasPeriod period, int timeoutMs);
	ctre::phoenix::ErrorCode ConfigVelocityMeasurementWindow(int windowSize,
			int timeoutMs);

	//------ limit switch ----------//
	ctre::phoenix::ErrorCode ConfigForwardLimitSwitchSource(
			LimitSwitchSource limitSwitchSource,
			LimitSwitchNormal normalOpenOrClose, int timeoutMs);
	ctre::phoenix::ErrorCode ConfigReverseLimitSwitchSource(
			LimitSwitchSource limitSwitchSource,
			LimitSwitchNormal normalOpenOrClose, int timeoutMs);
	ctre::phoenix::ErrorCode ConfigForwardLimitSwitchSource(
			RemoteLimitSwitchSource limitSwitchSource,
			LimitSwitchNormal normalOpenOrClose, int deviceID, int timeoutMs);
	ctre::phoenix::ErrorCode ConfigReverseLimitSwitchSource(
			RemoteLimitSwitchSource limitSwitchSource,
			LimitSwitchNormal normalOpenOrClose, int deviceID, int timeoutMs);

	//------ Current Limit ----------//
	ctre::phoenix::ErrorCode ConfigPeakCurrentLimit(int amps, int timeoutMs);
	ctre::phoenix::ErrorCode ConfigPeakCurrentDuration(int milliseconds,
			int timeoutMs);
	ctre::phoenix::ErrorCode ConfigContinuousCurrentLimit(int amps,
			int timeoutMs);
	void EnableCurrentLimit(bool enable);

private:
	TalonImpl* impl;
};

