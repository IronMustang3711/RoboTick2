/*
 * WPITalonAdapter.cpp
 *
 *  Created on: Jan 31, 2018
 *      Author: jason
 */

#include "WPITalonAdapter.h"
#include <ctre/Phoenix.h>

MyTalon::MyTalon(TalonImpl * _impl): impl{_impl} 
{
}

MyTalon::MyTalon(const MyTalon & other) : MyTalon{other.impl} //copy ctor
{
}

MyTalon::MyTalon(MyTalon && other) : MyTalon{ other.impl } //move ctor
{
}

MyTalon & MyTalon::operator=(const MyTalon & other) //copy assign
{
	this->impl = other.impl;
	return *this;
}

MyTalon & MyTalon::operator=(MyTalon && other) //move assign
{
	this->impl = other.impl;
	return *this;
}

MyTalon::~MyTalon()
{
	//if (impl != nullptr) delete impl;
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



