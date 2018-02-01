/*
 * WPITalonAdapter.cpp
 *
 *  Created on: Jan 31, 2018
 *      Author: jason
 */

#include "WPITalonAdapter.h"
#include <ctre/Phoenix.h>
WPITalonAdapter::WPITalonAdapter(Talon_t& talon): frc::SpeedController{}, delegate{talon}{
}

void WPITalonAdapter::PIDWrite(double output) {
	Set(output);
}

void WPITalonAdapter::Set(double speed) {
	delegate.Set(ControlMode::PercentOutput,speed);
}

double WPITalonAdapter::Get() const{
	return delegate.GetMotorOutputPercent();
}

void WPITalonAdapter::SetInverted(bool isInverted) {
	delegate.SetInverted(isInverted);
}

bool WPITalonAdapter::GetInverted() const {
	return delegate.GetInverted();
}

void WPITalonAdapter::Disable() {
	StopMotor();
}

void WPITalonAdapter::StopMotor() {
	delegate.Set(ControlMode::PercentOutput,0);
}
