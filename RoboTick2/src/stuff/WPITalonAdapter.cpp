/*
 * WPITalonAdapter.cpp
 *
 *  Created on: Jan 31, 2018
 *      Author: jason
 */

#include "WPITalonAdapter.h"
#include <ctre/Phoenix.h>


MyTalon::MyTalon(TalonImpl * imp): impl{imp}
{
}


void MyTalon::Set(double speed)
{
	impl->Set(ControlMode::PercentOutput, speed);
}

double MyTalon::Get() const
{
	return impl->GetMotorOutputPercent();
}

void MyTalon::SetInverted(bool isInverted)
{
	impl->SetInverted(isInverted);
}

bool MyTalon::GetInverted() const
{
	return impl->GetInverted();
}



