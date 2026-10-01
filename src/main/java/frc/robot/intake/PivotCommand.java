// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.units.Units.Degrees;

/** A pivot command that pivots the intake to a given position. */
public class PivotCommand extends Command {
    private double position;
    
    /**
     * Constructs a pivot command
     * @param position - The angle in degrees
     */
    public PivotCommand(double position) {
        setName("PivotCommand");
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(IntakeSubsystem.getInstance());
    
        this.position = position;
    }

    /**
     * Constructs a pivot command
     * @param position - The angle with the wrapper class {@link Angle}
     */
    public PivotCommand(Angle position) {
        this(position.in(Degrees));
    }

    @Override
    public void initialize() {
        IntakeSubsystem.getInstance().motionMagicPosition(this.position);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return IntakeSubsystem.getInstance().withinTolerance(this.position);
    }
}