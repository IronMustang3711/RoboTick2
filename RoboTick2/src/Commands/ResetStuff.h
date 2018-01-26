#pragma once

#include <WPILib.h>
#include "../Subsystems/DriveTrain.h"

class DrivetrainResetCommand: public InstantCommand {
protected:
	DrivetrainResetCommand(const llvm::Twine& name, DriveTrain& drive);
	DriveTrain& drive;

};

struct ResetEncoders: DrivetrainResetCommand {

	ResetEncoders(DriveTrain& drive);

	void Execute() override;
};

struct ResetGyro: DrivetrainResetCommand {

	ResetGyro(DriveTrain& drive);

	void Execute() override;
};

