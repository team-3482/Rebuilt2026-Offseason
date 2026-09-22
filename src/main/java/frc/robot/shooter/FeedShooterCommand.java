// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.ShooterConstants;
import frc.robot.swerve.SwerveSubsystem;

/** Feed fuel into shooter with Feeder and Sterilizer motors */
public class FeedShooterCommand extends Command {
    private final boolean doChecks;
    private final double direction;

    public FeedShooterCommand(boolean doChecks, boolean reversed) {
        setName("FeedShooterCommand");

        this.doChecks = doChecks;
        direction = reversed ? -1.0 : 1.0;
    }

    public FeedShooterCommand() {
        this(true, false);
    }

    @Override
    public void execute() {
        if (
            !doChecks
            || (ShooterSubsystem.getInstance().isShooterVelocityWithinTolerance()
            && SwerveSubsystem.getInstance().isAngleWithinToleranceToTarget())
        ) {
            ShooterSubsystem.getInstance().setSterilizerSpeed(ShooterConstants.STERILIZER_SPEED * direction);
            ShooterSubsystem.getInstance().setFeederSpeed(ShooterConstants.FEEDER_SPEED * direction);
        } else {
            ShooterSubsystem.getInstance().setFeederSpeed(0);
            ShooterSubsystem.getInstance().setSterilizerSpeed(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        ShooterSubsystem.getInstance().setFeederSpeed(0);
        ShooterSubsystem.getInstance().setSterilizerSpeed(0);
    }
}