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


class MyTalon : public frc::SpeedController {

	using TalonImpl = ctre::phoenix::motorcontrol::can::TalonSRX;

public:
	MyTalon(TalonImpl* impl);

	MyTalon(const MyTalon& other) = default;
	MyTalon(MyTalon&& other)      = default;

	MyTalon& operator= (const MyTalon& other) = default;
	MyTalon& operator= (MyTalon&& other)      = default;






	//PIDOutput
	inline void PIDWrite(double output) override { Set(output); }

	//SpeedController
	void Set(double speed) override;
	double Get() const override;
	void SetInverted(bool isInverted) override;
	bool GetInverted() const override;
	inline void Disable() override { StopMotor(); }
	inline void StopMotor() override { Set(0); }


private:
	TalonImpl* impl;
};


