// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.template;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that does nothing. */
public class ExampleCommand extends Command {
    public ExampleCommand() {
        setName("ExampleCommand");
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(ExampleSubsystem.getInstance());
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}