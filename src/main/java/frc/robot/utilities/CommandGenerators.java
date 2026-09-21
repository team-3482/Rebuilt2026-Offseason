package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Constants.CalculationConstants;
import frc.robot.constants.Constants.Positions;
import frc.robot.shooter.RevShooterCommand;
import frc.robot.swerve.LookAtPositionCommand;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.vision.ResetPoseCommand;
import org.littletonrobotics.junction.Logger;

/** Class that holds commands that don't need to clutter RobotContainer */
public class CommandGenerators {
    private static Command scheduledPrepareHubCommand;
    private static Command scheduledPrepareFerryCommand;

    /**
     * A command that cancels all running commands.
     * @return The command.
     */
    public static Command CancelAllCommands() {
        return Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll());
    }

    /* Helper functions */

    /**
     * Aligns to target, Aims the Hood, and then revs the Shooter. This command must be run in a previous command thread (.run, .runOnce, .runEnd)
     * @param target The target.
     * @return The command.
     */
    public static Command AimAndRevShooter(Pose2d target, boolean hub) {
        // TODO make this a lambda expression
        Distance distance = SwerveSubsystem.getInstance().getDistance(target);

        try {
            Logger.recordOutput("Shooter/Target", target);
        } catch (Exception ignored) {}

        // Get angle for move hood cmd
        // Angle angle = HoodSubsystem.getInstance().getShootingHoodAngle(distance, velocity, hub);

        // Schedule command
        Command parallelCommand = Commands.parallel(
            new LookAtPositionCommand(target),
            // new MoveHoodCommand(angle),
            new RevShooterCommand(target)
        );

        CommandScheduler.getInstance().schedule(parallelCommand);

        if (distance.gt(CalculationConstants.MAX_SHOOTING_DISTANCE)
            && distance.lt(CalculationConstants.MIN_SHOOTING_DISTANCE)) {
            System.out.println("Target out of range!!!");
            // Elastic.sendNotification(new Notification(NotificationLevel.ERROR, "AimAndRevShooter", "Target out of range!"));
        }

        return parallelCommand;
    }

    /**
     * A command that resets the odometry to an empty Pose2d.
     * @return The command.
     */
    public static Command ResetOdometryToOriginCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> SwerveSubsystem.getInstance().resetPose(Pose2d.kZero)),
            new ResetPoseCommand().withTimeout(0.25)
        );
    }

    /**
     * A command that takes the current orientation of the robot
     * and makes it X forward for field-relative maneuvers.
     * @return The command.
     */
    public static Command SetForwardDirectionCommand() {
        return Commands.runOnce(() -> SwerveSubsystem.getInstance().seedFieldCentric());
    }

    // /**
    //  * Continuously move the hood angle to meet a target's distance.
    //  * @param target The target.
    //  * @return The command.
    //  */
    // public static Command ContinuousMoveHoodCommand(Pose2d target, boolean hub) {
    //     return Commands.runOnce(() -> {
    //         Distance distance = SwerveSubsystem.getInstance().getDistance(target);
    //         LinearVelocity velocity = ShooterSubsystem.getInstance().getFuelLinearVelocity(distance);
    //         Logger.recordOutput("Shooter/FuelLinearVelocity", velocity.in(MetersPerSecond));
    //         Angle angle = HoodSubsystem.getInstance().getShootingHoodAngle(distance, velocity, hub);
    //         CommandScheduler.getInstance().schedule(new MoveHoodCommand(angle));
    //     });
    // }

    /**
     * Aims to our alliance side and revs shooter
     * @return The command.
     */
    public static Command PrepareFerry() {
        return Commands.runEnd(
            () -> {
                boolean redAlliance = DriverStation.getAlliance().orElse(Alliance.Blue).equals(DriverStation.Alliance.Red);
                boolean topHalf = SwerveSubsystem.getInstance().getState().Pose.getY() > Positions.HALF_FIELD_Y;

                Pose2d position = redAlliance
                    ? (topHalf ? Positions.RED_TOP_FERRY : Positions.RED_BOTTOM_FERRY)
                    : (topHalf ? Positions.BLUE_TOP_FERRY : Positions.BLUE_BOTTOM_FERRY);

                scheduledPrepareFerryCommand = AimAndRevShooter(position, false);
            },
            () -> {
                scheduledPrepareFerryCommand.cancel();
            });
    }

    /**
     * Aims to the Hub and revs the Shooter
     * @return The command.
     */
    public static Command PrepareHub() {
        return Commands.runEnd(
            () -> {
                boolean redAlliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue).equals(DriverStation.Alliance.Red);
                Pose2d target = redAlliance ? Positions.RED_HUB : Positions.BLUE_HUB;
                scheduledPrepareHubCommand = AimAndRevShooter(target, true);
            },
            () -> {
                scheduledPrepareHubCommand.cancel();
            });
    }
}