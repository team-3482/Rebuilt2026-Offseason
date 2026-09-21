// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hood;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;

/** Move shooter hood to position */
public class MoveHoodCommand extends Command {
    private final double position;

    public MoveHoodCommand(double position) {
        setName("MoveHoodCommand");
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(HoodSubsystem.getInstance());

        this.position = position;
    }

    public MoveHoodCommand(Angle angle) {
        setName("MoveHoodCommand");
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(HoodSubsystem.getInstance());

        this.position = HoodSubsystem.getInstance().hoodAngleToDouble(angle);
    }

    @Override
    public void initialize() {
        HoodSubsystem.getInstance().setHoodPosition(position);
    }

    @Override
    public boolean isFinished() {
        return HoodSubsystem.getInstance().isPositionWithinTolerance();
    }
}