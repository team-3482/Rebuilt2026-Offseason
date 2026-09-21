// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.template;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** An example subsystem that does nothing. */
public class ExampleSubsystem extends SubsystemBase {
    // Use Bill Pugh Singleton Pattern for efficient lazy initialization (thread-safe !)
    private static class ExampleSubsystemHolder {
        private static final ExampleSubsystem INSTANCE = new ExampleSubsystem();
    }

    /** Always use this method to get the singleton instance of this subsystem. */
    public static ExampleSubsystem getInstance() {
        return ExampleSubsystemHolder.INSTANCE;
    }

    private ExampleSubsystem() {
        super("ExampleSubsystem");
    }

    @Override
    public void periodic() {}
}