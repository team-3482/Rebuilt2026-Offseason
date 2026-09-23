package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.Constants.CalculationConstants;
import frc.robot.constants.Constants.Positions;
import frc.robot.shooter.RevShooterCommand;
import frc.robot.swerve.LookAtPositionCommand;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.vision.ResetPoseCommand;

import java.util.Set;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

/** Class that holds commands that don't need to clutter RobotContainer */
public class CommandGenerators {
    private static Set<Subsystem> cachedAimAndRevRequirements;

    /**
     * A command that cancels all running commands.
     * @return The command.
     */
    public static Command CancelAllCommands() {
        return Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll());
    }

    /* Helper functions */

    /**
     * Returns the command to align to target and rev the Shooter
     * @param target The target.
     * @return The parallel command
     */
    private static Command AimAndRevShooter(Pose2d target) {
        Distance distance = SwerveSubsystem.getInstance().getDistance(target);

        try {
            Logger.recordOutput("Shooter/Target", target);
        } catch (Exception ignored) {}

        if (distance.gt(CalculationConstants.MAX_SHOOTING_DISTANCE)
            && distance.lt(CalculationConstants.MIN_SHOOTING_DISTANCE)) {
            System.out.println("Target out of range!!!");
            // Elastic.sendNotification(new Notification(NotificationLevel.ERROR, "AimAndRevShooter", "Target out of range!"));
        }

        Command parallelCommand = Commands.parallel(
            new LookAtPositionCommand(target),
            new RevShooterCommand(target)
        );
        return parallelCommand;
    }

    /**
     * Handles the logic for deferring aim and rev shooter by wrapping all logic around a lambda that requires a pose
     * @param targetSupplier The supplier / lambda for the pose2d (target)
     * @return The final aim and rev command
     */
    private static Command HandleAimAndRevShooter(Supplier<Pose2d> targetSupplier) {
    if (cachedAimAndRevRequirements == null) {
        // Build one throwaway instance just to harvest requirements for defer().
        cachedAimAndRevRequirements = AimAndRevShooter(Pose2d.kZero).getRequirements();
    }

    return Commands.defer(
        () -> AimAndRevShooter(targetSupplier.get()),
        cachedAimAndRevRequirements
    );
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

    /**
     * Aims to our alliance side and revs shooter
     * @return The command.
     */
    public static Command PrepareFerry() {
        return HandleAimAndRevShooter(() -> {
            boolean redAlliance = DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
            boolean topHalf = SwerveSubsystem.getInstance().getState().Pose.getY() > Positions.HALF_FIELD_Y;

            return redAlliance
                ? (topHalf ? Positions.RED_TOP_FERRY : Positions.RED_BOTTOM_FERRY)
                : (topHalf ? Positions.BLUE_TOP_FERRY : Positions.BLUE_BOTTOM_FERRY);
        });
    }

    /**
     * Aims to the Hub and revs the Shooter
     * @return The command.
     */
    public static Command PrepareHub() {
         return HandleAimAndRevShooter(() -> {
            boolean redAlliance = DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
            return redAlliance ? Positions.RED_HUB : Positions.BLUE_HUB;
        });
    }
}