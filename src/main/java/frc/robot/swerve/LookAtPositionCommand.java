// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants.AutoAngleConstants;
import frc.robot.constants.TunerConstants;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

/** Takes a position on the field and automatically rotates to face it */
public class LookAtPositionCommand extends Command {
    final Pose2d target;

    public LookAtPositionCommand(Pose2d target) {
        setName("LookAtPositionCommand");

        this.target = target;

        controller.enableContinuousInput(-Math.PI, Math.PI);
        controller.setTolerance(AutoAngleConstants.TOLERANCE.in(Radians));

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(SwerveSubsystem.getInstance());
    }

    SwerveDriveState state;
    double angleToTarget;
    double xDistance;
    double yDistance;

    private final SwerveRequest.FieldCentricFacingAngle facingAngleDrive = new SwerveRequest.FieldCentricFacingAngle()
        .withDeadband(TunerConstants.kSpeedAt12Volts.magnitude() * 0.035)
        .withDriveRequestType(DriveRequestType.Velocity);

    private final PhoenixPIDController controller = new PhoenixPIDController(AutoAngleConstants.P, AutoAngleConstants.I, AutoAngleConstants.D);
    boolean flip;

    @Override
    public void initialize() {
        state = SwerveSubsystem.getInstance().getState();

        controller.reset();
        facingAngleDrive.HeadingController = controller;

        flip = RobotContainer.getInstance().driverController_HID.getLeftTriggerAxis() >= 0.5;

        boolean redAlliance = DriverStation.getAlliance().orElse(Alliance.Blue).equals(DriverStation.Alliance.Red);
        if (redAlliance) { flip = !flip; }

        calculateAngle();
    }

    @Override
    public void execute() {
        SwerveSubsystem.getInstance().setControl(facingAngleDrive
            .withVelocityX(0)
            .withVelocityY(0)
            .withTargetDirection(new Rotation2d(angleToTarget))
        );

        SwerveSubsystem.getInstance().setTargetAngle(Radians.of(angleToTarget));
        Pose2d targetPose = new Pose2d(state.Pose.getTranslation(), new Rotation2d(angleToTarget));
        try {
            Logger.recordOutput("Shooter/TargetPose", targetPose);
        } catch (Exception ignored) {}

        try {
            Logger.recordOutput("Shooter/Target", target);
        } catch (Exception ignored) {}
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println("Swerve Auto Angle command interrupted.");
        } else {
            System.out.println("Swerve angle within tolerance.");
        }
    }

    @Override
    public boolean isFinished() {
        return controller.atSetpoint();
    }

    /**
     * Calculate the angle that for the bot facing the target
     */
    private void calculateAngle() {
        xDistance = target.getMeasureX().in(Meters) - state.Pose.getMeasureX().in(Meters);
        yDistance = target.getMeasureY().in(Meters) - state.Pose.getMeasureY().in(Meters);

        Angle angle = Radians.of(Math.atan2(yDistance, xDistance));

        if(angle.in(Degrees) > 180) {
            angle = angle.minus(Degrees.of(180));
            angle = Degrees.of(-180).plus(angle);
        } else if (angle.in(Degrees) < -180) {
            angle = angle.plus(Degrees.of(180));
            angle = Degrees.of(180).plus(angle);
        }

        angleToTarget = angle.in(Radians);

        if(flip) { angleToTarget -= Math.PI; }
        System.out.println("target angle: " + angleToTarget);
    }
}
