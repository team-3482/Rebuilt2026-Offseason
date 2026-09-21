// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.wpilibj2.command.Command;

/** Reset the QuestNav pose based on absolute Limelight data */
public class ResetPoseCommand extends Command {
    private boolean reset = false;

    public ResetPoseCommand() {
        setName("ResetPoseCommand");
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(VisionSubsystem.getInstance());
    }

    @Override
    public void execute() {
        if (VisionSubsystem.getInstance().trustLimelightData()) {
            VisionSubsystem.getInstance().resetPose();
            reset = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            System.out.println("QuestNav pose reset from Limelight AprilTag data!");
        } else {
            System.out.println("Failed to reset QuestNav pose, no AprilTag in view!");
        }
    }

    @Override
    public boolean isFinished() {
        return reset;
    }
}