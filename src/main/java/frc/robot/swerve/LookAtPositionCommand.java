// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.AutoAngleConstants;

import static edu.wpi.first.units.Units.*;

/** Takes a position on the field and automatically rotates to face it */
public class LookAtPositionCommand extends Command {
    final Pose2d target;
    // The target angle to rotate to, in rad 
    private double targetAngle = 0;

    private final PIDController turnPID = new PIDController(AutoAngleConstants.P, AutoAngleConstants.I, AutoAngleConstants.D);
    private final SwerveRequest.ApplyRobotSpeeds drive = new SwerveRequest.ApplyRobotSpeeds();

    public LookAtPositionCommand(Pose2d target) {
        setName("LookAtPositionCommand");

        this.target = target;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(SwerveSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        Pose2d pose = SwerveSubsystem.getInstance().getState().Pose;
        this.targetAngle = normalizeAngle(Math.atan2(this.target.getY() - pose.getY(), this.target.getX() - pose.getX()));
        SwerveSubsystem.getInstance().setTargetAngle(this.targetAngle);
    }

    @Override
    public void execute() {
        Pose2d pose = SwerveSubsystem.getInstance().getState().Pose;
        double currentAngle = pose.getRotation().getRadians();
        double error = targetAngle - currentAngle;

        double output = turnPID.calculate(error);
        System.out.println(output);
        SwerveSubsystem.getInstance().setControl(
            drive.withSpeeds(new ChassisSpeeds(0, 0, output))
        );
    }

    @Override
    public void end(boolean interrupted) {
        SwerveSubsystem.getInstance().setControl(
            drive.withSpeeds(new ChassisSpeeds())
        );
    }

    @Override
    public boolean isFinished() {
        Pose2d pose = SwerveSubsystem.getInstance().getState().Pose;
        double currentAngle = normalizeAngle(pose.getRotation().getRadians());
        double error = targetAngle - currentAngle;

        boolean inTolerance = Math.abs(error) < AutoAngleConstants.TOLERANCE.in(Radians);
        SwerveSubsystem.getInstance().setAngleWithinToleranceToTarget(inTolerance);
        return inTolerance;
    }
    
    /**
     * Normalizes an angle to the range [0, 2pi].
     * @param angle - Angle as radians
     * @return Normalized angle in radians
     */
    private double normalizeAngle(double angle) {
        double result = angle % (2 * Math.PI);
        if (result < 0) {
            result += 2 * Math.PI;
        }
        return result;
    }
}
