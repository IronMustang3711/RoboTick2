#pragma once
#include <SpeedController.h>

namespace ctre {
namespace phoenix {
namespace motorcontrol {
namespace can {
class TalonSRX;
}
}
}
}

class WPITalonAdapter: public frc::SpeedController {
public:
	using Talon_t = ctre::phoenix::motorcontrol::can::TalonSRX;

	WPITalonAdapter(Talon_t& talon);

	//PIDOutput
	void PIDWrite(double output) override;

	//SpeedController
	void Set(double speed) override;
	double Get() const override;
	void SetInverted(bool isInverted) override;
	bool GetInverted() const override;
	void Disable() override;
	void StopMotor() override;

private:
	Talon_t& delegate;
};

class MyTalon {

	using TalonImpl = ctre::phoenix::motorcontrol::can::TalonSRX;

public:
	MyTalon(TalonImpl* impl);
	MyTalon(const MyTalon& other);
	MyTalon(MyTalon&& other);

	MyTalon& operator =(const MyTalon& other);
	MyTalon& operator =(MyTalon&& other);

	~MyTalon();

private:
	TalonImpl* impl;
};


