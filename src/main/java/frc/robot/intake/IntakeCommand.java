// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.IntakeConstants;

/** An example command that does nothing. */
public class IntakeCommand extends Command {
    public IntakeCommand() {
        setName("IntakeCommand");
    }

    @Override
    public void initialize() {
        IntakeSubsystem.getInstance().setIntakeSpeed(IntakeConstants.INTAKE_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        IntakeSubsystem.getInstance().setIntakeSpeed(0);
    }
}