/*
 * WPITalonAdapter.cpp
 *
 *  Created on: Jan 31, 2018
 *      Author: jason
 */

#include "WPITalonAdapter.h"
#include <ctre/Phoenix.h>

MyTalon::MyTalon(TalonImpl * imp) :
		impl { imp } {
	assert(impl != nullptr);
}

void MyTalon::Set(double speed) {
	impl->Set(ControlMode::PercentOutput, speed);
}

double MyTalon::Get() const {
	return impl->GetMotorOutputPercent();
}

void MyTalon::SetInverted(bool isInverted) {
	impl->SetInverted(isInverted);
}
bool MyTalon::GetInverted() const {
	return impl->GetInverted();
}

int MyTalon::GetDeviceID() {
	return impl->GetDeviceID();
}

void MyTalon::Set(ControlMode Mode, double value) {
	impl->Set(Mode,value);
}

void MyTalon::Set(ControlMode mode, double demand0, double demand1) {
	impl->Set(mode,demand0,demand1);
}

void MyTalon::NeutralOutput() {
	impl->NeutralOutput();
}

void MyTalon::SetNeutralMode(NeutralMode neutralMode) {
	impl->SetNeutralMode(neutralMode);
}

void MyTalon::EnableHeadingHold(bool enable) {
	impl->EnableHeadingHold(enable);
}

void MyTalon::SelectDemandType(bool value) {
	impl->SelectDemandType(value);
}

void MyTalon::SetSensorPhase(bool PhaseSensor) {
	impl->SetSensorPhase(PhaseSensor);
}

ctre::phoenix::ErrorCode MyTalon::ConfigOpenloopRamp(
		double secondsFromNeutralToFull, int timeoutMs) {
	return impl->ConfigOpenloopRamp(secondsFromNeutralToFull, timeoutMs);
}

ctre::phoenix::ErrorCode MyTalon::ConfigClosedloopRamp(
		double secondsFromNeutralToFull, int timeoutMs) {
	return impl->ConfigClosedloopRamp(secondsFromNeutralToFull, timeoutMs);
}

ctre::phoenix::ErrorCode MyTalon::ConfigPeakOutputForward(double percentOut,
		int timeoutMs) {
	return impl->ConfigPeakOutputForward(percentOut, timeoutMs);
}

ctre::phoenix::ErrorCode MyTalon::ConfigPeakOutputReverse(double percentOut,
		int timeoutMs) {
	return impl->ConfigPeakOutputReverse(percentOut, timeoutMs);
}

ctre::phoenix::ErrorCode MyTalon::ConfigNominalOutputForward(double percentOut,
		int timeoutMs) {
	return impl->ConfigNominalOutputForward(percentOut, timeoutMs);
}

ctre::phoenix::ErrorCode MyTalon::ConfigNominalOutputReverse(double percentOut,
		int timeoutMs) {
	return impl->ConfigNominalOutputReverse(percentOut, timeoutMs);
}

ctre::phoenix::ErrorCode MyTalon::ConfigNeutralDeadband(double percentDeadband,
		int timeoutMs) {
	return impl->ConfigNeutralDeadband(percentDeadband, timeoutMs);
}

ctre::phoenix::ErrorCode MyTalon::ConfigVoltageCompSaturation(double voltage,
		int timeoutMs) {
	return impl->ConfigVoltageCompSaturation(voltage, timeoutMs);
}

ctre::phoenix::ErrorCode MyTalon::ConfigVoltageMeasurementFilter(
		int filterWindowSamples, int timeoutMs) {
	return impl->ConfigVoltageMeasurementFilter(filterWindowSamples, timeoutMs);
}

void MyTalon::EnableVoltageCompensation(bool enable) {
	impl->EnableVoltageCompensation(enable);
}

double MyTalon::GetBusVoltage() {
	return impl->GetBusVoltage();
}

double MyTalon::GetMotorOutputPercent() {
	return impl->GetMotorOutputPercent();
}

double MyTalon::GetMotorOutputVoltage() {
	return impl->GetMotorOutputVoltage();
}

double MyTalon::GetOutputCurrent() {
	return impl->GetOutputCurrent();
}

double MyTalon::GetTemperature() {
	return impl->GetTemperature();
}

ctre::phoenix::ErrorCode MyTalon::ConfigSelectedFeedbackSensor(
		RemoteFeedbackDevice feedbackDevice, int pidIdx, int timeoutMs) {
	return impl->ConfigSelectedFeedbackSensor(feedbackDevice,pidIdx, timeoutMs);
}

ctre::phoenix::ErrorCode MyTalon::ConfigSelectedFeedbackSensor(
		FeedbackDevice feedbackDevice, int pidIdx, int timeoutMs) {
}

ctre::phoenix::ErrorCode MyTalon::ConfigRemoteFeedbackFilter(int deviceID,
		RemoteSensorSource remoteSensorSource, int remoteOrdinal,
		int timeoutMs) {
}

ctre::phoenix::ErrorCode MyTalon::ConfigSensorTerm(SensorTerm sensorTerm,
		FeedbackDevice feedbackDevice, int timeoutMs) {
}

int MyTalon::GetSelectedSensorPosition(int pidIdx) {
}

int MyTalon::GetSelectedSensorVelocity(int pidIdx) {
}

ctre::phoenix::ErrorCode MyTalon::SetSelectedSensorPosition(int sensorPos,
		int pidIdx, int timeoutMs) {
}

ctre::phoenix::ErrorCode MyTalon::SetControlFramePeriod(ControlFrame frame,
		int periodMs) {
}

ctre::phoenix::ErrorCode MyTalon::SetStatusFramePeriod(StatusFrame frame,
		int periodMs, int timeoutMs) {
}

ctre::phoenix::ErrorCode MyTalon::SetStatusFramePeriod(
		StatusFrameEnhanced frame, int periodMs, int timeoutMs) {
}

int MyTalon::GetStatusFramePeriod(StatusFrame frame, int timeoutMs) {
}

int MyTalon::GetStatusFramePeriod(StatusFrameEnhanced frame, int timeoutMs) {
}

ctre::phoenix::ErrorCode MyTalon::ConfigVelocityMeasurementPeriod(
		VelocityMeasPeriod period, int timeoutMs) {
}

ctre::phoenix::ErrorCode MyTalon::ConfigForwardLimitSwitchSource(
		RemoteLimitSwitchSource type, LimitSwitchNormal normalOpenOrClose,
		int deviceID, int timeoutMs) {
}

ctre::phoenix::ErrorCode MyTalon::ConfigReverseLimitSwitchSource(
		RemoteLimitSwitchSource type, LimitSwitchNormal normalOpenOrClose,
		int deviceID, int timeoutMs) {
}

void MyTalon::OverrideLimitSwitchesEnable(bool enable) {
}

ctre::phoenix::ErrorCode MyTalon::ConfigForwardLimitSwitchSource(
		LimitSwitchSource type, LimitSwitchNormal normalOpenOrClose,
		int timeoutMs) {
}

ctre::phoenix::ErrorCode MyTalon::ConfigReverseLimitSwitchSource(
		LimitSwitchSource type, LimitSwitchNormal normalOpenOrClose,
		int timeoutMs) {
}

ctre::phoenix::ErrorCode MyTalon::ConfigForwardSoftLimitThreshold(
		int forwardSensorLimit, int timeoutMs) {
}

ctre::phoenix::ErrorCode MyTalon::ConfigReverseSoftLimitThreshold(
		int reverseSensorLimit, int timeoutMs) {
}

ctre::phoenix::ErrorCode MyTalon::ConfigForwardSoftLimitEnable(bool enable,
		int timeoutMs) {
}

ctre::phoenix::ErrorCode MyTalon::ConfigReverseSoftLimitEnable(bool enable,
		int timeoutMs) {
}

void MyTalon::OverrideSoftLimitsEnable(bool enable) {
}

ctre::phoenix::ErrorCode MyTalon::ConfigVelocityMeasurementWindow(
		int windowSize, int timeoutMs) {
}

ctre::phoenix::ErrorCode MyTalon::Config_kP(int slotIdx, double value,
		int timeoutMs) {
}

ctre::phoenix::ErrorCode MyTalon::Config_kI(int slotIdx, double value,
		int timeoutMs) {
}

ctre::phoenix::ErrorCode MyTalon::Config_kD(int slotIdx, double value,
		int timeoutMs) {
}

ctre::phoenix::ErrorCode MyTalon::Config_kF(int slotIdx, double value,
		int timeoutMs) {
}

ctre::phoenix::ErrorCode MyTalon::Config_IntegralZone(int slotIdx, int izone,
		int timeoutMs) {
}

ctre::phoenix::ErrorCode MyTalon::ConfigAllowableClosedloopError(int slotIdx,
		int allowableCloseLoopError, int timeoutMs) {
}

ctre::phoenix::ErrorCode MyTalon::ConfigMaxIntegralAccumulator(int slotIdx,
		double iaccum, int timeoutMs) {
}

ctre::phoenix::ErrorCode MyTalon::SetIntegralAccumulator(double iaccum,
		int pidIdx, int timeoutMs) {
}

int MyTalon::GetClosedLoopError(int pidIdx) {
}

double MyTalon::GetIntegralAccumulator(int pidIdx) {
}

double MyTalon::GetErrorDerivative(int pidIdx) {
}

ctre::phoenix::ErrorCode MyTalon::SelectProfileSlot(int slotIdx, int pidIdx) {
}

int MyTalon::GetClosedLoopTarget(int pidIdx) {
}

int MyTalon::GetActiveTrajectoryPosition() {
}

int MyTalon::GetActiveTrajectoryVelocity() {
}

double MyTalon::GetActiveTrajectoryHeading() {
}

ctre::phoenix::ErrorCode MyTalon::ConfigMotionCruiseVelocity(
		int sensorUnitsPer100ms, int timeoutMs) {
}

ctre::phoenix::ErrorCode MyTalon::ConfigMotionAcceleration(
		int sensorUnitsPer100msPerSec, int timeoutMs) {
}

void MyTalon::ClearMotionProfileTrajectories() {
}

int MyTalon::GetMotionProfileTopLevelBufferCount() {
}

ctre::phoenix::ErrorCode MyTalon::PushMotionProfileTrajectory(
		const ctre::phoenix::motion::TrajectoryPoint& trajPt) {
}

bool MyTalon::IsMotionProfileTopLevelBufferFull() {
}

void MyTalon::ProcessMotionProfileBuffer() {
}

ctre::phoenix::ErrorCode MyTalon::GetMotionProfileStatus(
		ctre::phoenix::motion::MotionProfileStatus& statusToFill) {
}

ctre::phoenix::ErrorCode MyTalon::ClearMotionProfileHasUnderrun(int timeoutMs) {
}

ctre::phoenix::ErrorCode MyTalon::ChangeMotionControlFramePeriod(int periodMs) {
}

ctre::phoenix::ErrorCode MyTalon::ConfigMotionProfileTrajectoryPeriod(
		int baseTrajDurationMs, int timeoutMs) {
}

ctre::phoenix::ErrorCode MyTalon::GetLastError() {
}

ctre::phoenix::ErrorCode MyTalon::GetFaults(Faults& toFill) {
}

ctre::phoenix::ErrorCode MyTalon::GetStickyFaults(StickyFaults& toFill) {
}

ctre::phoenix::ErrorCode MyTalon::ClearStickyFaults(int timeoutMs) {
}

int MyTalon::GetFirmwareVersion() {
}

bool MyTalon::HasResetOccurred() {
}

ctre::phoenix::ErrorCode MyTalon::ConfigSetCustomParam(int newValue,
		int paramIndex, int timeoutMs) {
}

int MyTalon::ConfigGetCustomParam(int paramIndex, int timeoutMs) {
}

ctre::phoenix::ErrorCode MyTalon::ConfigSetParameter(
		ctre::phoenix::ParamEnum param, double value, uint8_t subValue,
		int ordinal, int timeoutMs) {
}

double MyTalon::ConfigGetParameter(ctre::phoenix::ParamEnum param, int ordinal,
		int timeoutMs) {
}

int MyTalon::GetBaseID() {
}

ControlMode MyTalon::GetControlMode() {
}

void MyTalon::ValueUpdated() {
}

ctre::phoenix::motorcontrol::SensorCollection& MyTalon::GetSensorCollection() {
}

//ctre::phoenix::ErrorCode MyTalon::ConfigForwardLimitSwitchSource(
//		LimitSwitchSource limitSwitchSource,
//		LimitSwitchNormal normalOpenOrClose, int timeoutMs) {
//}
//
//ctre::phoenix::ErrorCode MyTalon::ConfigReverseLimitSwitchSource(
//		LimitSwitchSource limitSwitchSource,
//		LimitSwitchNormal normalOpenOrClose, int timeoutMs) {
//}
//
//ctre::phoenix::ErrorCode MyTalon::ConfigForwardLimitSwitchSource(
//		RemoteLimitSwitchSource limitSwitchSource,
//		LimitSwitchNormal normalOpenOrClose, int deviceID, int timeoutMs) {
//}
//
//ctre::phoenix::ErrorCode MyTalon::ConfigReverseLimitSwitchSource(
//		RemoteLimitSwitchSource limitSwitchSource,
//		LimitSwitchNormal normalOpenOrClose, int deviceID, int timeoutMs) {
//}

ctre::phoenix::ErrorCode MyTalon::ConfigPeakCurrentLimit(int amps,
		int timeoutMs) {
	return impl->ConfigPeakCurrentLimit(amps, timeoutMs);
}

ctre::phoenix::ErrorCode MyTalon::ConfigPeakCurrentDuration(int milliseconds,
		int timeoutMs) {
	return impl->ConfigPeakCurrentDuration(milliseconds, timeoutMs);
}

ctre::phoenix::ErrorCode MyTalon::ConfigContinuousCurrentLimit(int amps,
		int timeoutMs) {
	return impl->ConfigContinuousCurrentLimit(amps, timeoutMs);
}

void MyTalon::EnableCurrentLimit(bool enable) {
	impl->EnableCurrentLimit(enable);
}
