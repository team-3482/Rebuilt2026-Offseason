// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.climb;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.ClimbConstants;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Degrees;

/** Climb from floor to tower */
public class ClimbCommand extends Command {
    public ClimbCommand() {
        setName("ClimbCommand");
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(ClimbSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        ClimbSubsystem.getInstance().setClimbPosition(ClimbConstants.CLIMB_POSITION);
    }

    @Override
    public void execute() {
        Logger.recordOutput("Climb/Position", ClimbSubsystem.getInstance().getClimbPosition().in(Units.Revolutions));
    }

    @Override
    public void end(boolean interrupted) {
        ClimbSubsystem.getInstance().setClimbPosition(Degrees.of(0));
    }

    @Override
    public boolean isFinished() {
        return false;//ClimbSubsystem.getInstance().withinTolerance(ClimbConstants.CLIMB_POSITION);
    }
}