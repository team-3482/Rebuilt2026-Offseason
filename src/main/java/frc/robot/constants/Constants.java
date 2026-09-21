package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

public class Constants {
    /** General specific constants about the robot. */
    public static final class RobotConstants {
        /** CAN Bus for all CTRE devices */
        public static final CANBus CAN_BUS = new CANBus("ctre");
    }

    /** Constants for the controller and dashboard */
    public static final class DriverStationConstants {
        /** DriverStation ID of the driver controller. */
        public static final int DRIVER_CONTROLLER_ID = 0;
        /** DriverStation ID of the operator controller. */
        public static final int OPERATOR_CONTROLLER_ID = 1;

        /** Removes input around the joystick's center (eliminates stick drift). */
        public static final double DEADBAND = 0.075;
        /** Speed multiplier when using fine-control movement. */
        public static final double FINE_CONTROL_MULT = 0.15;

        /** Tab to show while humans are driving */
        public static final String TELEOP_TAB = "Teleoperated";
        /** Tab to show during autonomous period at beginning of match */
        public static final String AUTON_TAB = "Autonomous";
        /** Extra tab for debugging/development */
        public static final String DEV_TAB = "Dev";
    }

    /** Constants for QuestNav and Limelight */
    public static final class VisionConstants {
        /** The name of the limelight */
        public static final String LIMELIGHT = "limelight-euryale";

        /** Quest mounting offsets */
        public static final Transform3d ROBOT_TO_QUEST = new Transform3d(
            new Translation3d( // values generated from https://www.desmos.com/calculator/ymzqtr2lip
                -0.22905289, // X -> Forward from robot center
                -0.02426006, // Y -> Left from robot center
                0 // Z -> Up from robot center
            ),
            new Rotation3d(
                Degrees.of(0), // Roll -> Counter-clockwise rotation on the X axis
                Degrees.of(0), // Pitch -> Counter-clockwise rotation on the Y axis
                Degrees.of(180) // Yaw -> Counter-clockwise rotation on the Z axis
            )
        );

        /** Standard Deviations for Trust with QuestNav */
        public static final Matrix<N3, N1> QUESTNAV_TRUST_STD_DEVS = VecBuilder.fill(
            0.02, // Trust down to 2cm in X direction
            0.02, // Trust down to 2cm in Y direction
            Units.degreesToRadians(2) // Trust down to 2 degrees rotational
        );

        /** Standard Deviations for Trust with QuestNav */
        public static final Matrix<N3, N1> LIMELIGHT_TRUST_STD_DEVS = VecBuilder.fill(
            0.75, // Trust down to 75cm in X direction
            0.75, // Trust down to 75cm in Y direction
            9999999 // Trust all rotational data
        );

        /** Maximum allowed AprilTag ambiguity value */
        public static final double MAX_APRILTAG_AMBIGUITY = 1.0;
    }

    /** Constants for intake subsystem (both the pivot and actual intake) */
    public static final class IntakeConstants {
        /** The CAN ID for the Left Pinion TalonFX */
        public static final int LEFT_PINION_MOTOR = 22;
        /** The CAN ID for the Right Pinion TalonFX */
        public static final int RIGHT_PINION_MOTOR = 26;
        /** The CAN ID for the Left Intake TalonFX */
        public static final int LEFT_INTAKE_MOTOR = 27;
            /** The CAN ID for the Right Intake TalonFX */
        public static final int RIGHT_INTAKE_MOTOR = 28;

        /** Minimum angle (On hardstop inside robot) */
        public static final Angle MINIMUM_POSITION = Rotations.of(0);
        /** Maximum angle (Intaking position) */
        public static final Angle MAXIMUM_POSITION = Rotations.of(3.125);

        /** Gear ratio for mechanism */
        public static final double ROTOR_TO_MECHANISM_RATIO = ((double) 30 / 12) * ((double) 36 / 19);

        // TODO: we can probably optimize this a bit
        /* Motion Magic Config */
        public static final double CRUISE_SPEED = 4;
        public static final double ACCELERATION = 8;

        /** Gains used for Motion Magic slot 0. */
        @SuppressWarnings("HungarianNotationConstants")
        public static final class Slot0Gains {
            public static final double kG = 0;
            public static final double kS = 0.22;
            public static final double kV = 0;
            public static final double kA = 0;
            public static final double kP = 32;
            public static final double kI = 0;
            public static final double kD = 0;
        }

        /** The tolerance used for pivot in degrees */
        public static final double PINION_TOLERANCE = 2;

        /** Speed to run the intake motor at */
        public static final double INTAKE_SPEED = 0.4;
    }

    /** Constants for the Shooter subsystem */
    public static final class ShooterConstants {
        /** CAN ID for Shooter motor 1 */
        public static final int SHOOTER_MOTOR_1 = 23;
        /** CAN ID for Shooter motor 2 */
        public static final int SHOOTER_MOTOR_2 = 21;
        /** CAN ID for Shooter motor 3 */
        public static final int SHOOTER_MOTOR_3 = 24;
        /** CAN ID for Feeder motor */
        public static final int FEEDER_MOTOR = 25;
        /** CAN ID for Sterilizer motor */
        public static final int STERILIZER_MOTOR = 30;

        /** Feed shooter debounce time so all balls consistently make it in */
        public static final double FEEDER_PAUSE_DURATION = 0.15;
        public static final double FEEDER_FEED_DURATION = 0.075;

        /** Feeder motor speed */
        public static final double FEEDER_SPEED = 1;
        /** Sterilizer motor speed */
        public static final double STERILIZER_SPEED = 0.4;

        /** Shooter velocity (RPM) threshold to start feeding fuel */
        public static final AngularVelocity VELOCITY_TOLERANCE = RPM.of(200);

        /** 1 motor rotation is 2 wheel rotations */
        public static final double GEAR_RATIO = (double) 1/2;

        /** Gains used for Motion Magic slot 0. */
        @SuppressWarnings("HungarianNotationConstants")
        public static final class Slot0Gains {
            public static final double kS = 0.19; // static friction
            public static final double kV = 0.055; // voltage required for 1 rot/s
            public static final double kP = 0.2; // voltage amount to correct for error of 1 rot/s
            public static final double kI = 0.05;
            public static final double kD = 0;
        }
    }

    /** Constants for the Hood subsystem */
    public static final class HoodConstants {
        /** PWM Channel for Hood Linear Actuator 1 */
        public static final int HOOD_SERVO_1 = 0;
        /** PWM Channel for Hood Linear Actuator 2 */
        public static final int HOOD_SERVO_2 = 1;

        /** Full Servo Length */
        public static final Distance SERVO_LENGTH = Millimeters.of(50);
        /** Max Servo speed to run at */
        public static final LinearVelocity MAX_SERVO_SPEED = Millimeters.of(32).per(Second);
        /** Hood angle minimum shooting angle (linear actuators fully extended) */
        public static final Angle HOOD_ANGLE_MIN = Degrees.of(54.2);
        /** Hood angle minimum shooting angle (linear actuators fully retracted) */
        public static final Angle HOOD_ANGLE_MAX = Degrees.of(69);
    }

    /** Constants for kinematics math */
    public static final class CalculationConstants {
        // https://engineerexcel.com/planetary-gear-calculation/
        // https://www.desmos.com/calculator/qwhpu9mivi
        /** Shooter wheel diameter */
        public static final Distance WHEEL_DIAMETER = Inches.of(3.965);
        /** Fuel ball diameter */
        public static final Distance FUEL_DIAMETER = Inches.of(5.91);
        /** Diameter of the hood (if it was a full circle) */
        public static final Distance RING_DIAMETER = Inches.of(15.155);

        /** Ratio of the diameter of the Ring to the diameter of the Wheel */
        public static final double GEAR_RATIO = RING_DIAMETER.in(Inches) / WHEEL_DIAMETER.in(Inches) + 1; // 4.822
        /** Ratio for converting Wheel angular velocity to Fuel angular velocity */
        public static final double WHEEL_TO_FUEL_ANGULAR_VELOCITY_RATIO = 1 / GEAR_RATIO; // 0.2074
        /** Radius of the path that the Fuel takes */
        public static final Distance CARRIER_RADIUS = Inches.of((FUEL_DIAMETER.in(Inches) / 2) + (WHEEL_DIAMETER.in(Inches) / 2)); // 0.1254 m
        /** Ratio for converting Motor angular velocity to Fuel linear velocity */
        public static final double SHOOTER_ANGULAR_TO_FUEL_LINEAR_VELOCITY_RATIO = WHEEL_TO_FUEL_ANGULAR_VELOCITY_RATIO * CARRIER_RADIUS.in(Meters); // 0.026
        /** Ratio for converting Fuel linear velocity to Motor angular velocity */
        public static final double FUEL_LINEAR_TO_SHOOTER_ANGULAR_VELOCITY_RATIO = 1 / SHOOTER_ANGULAR_TO_FUEL_LINEAR_VELOCITY_RATIO; // 38.46

        /** Acceleration of gravity */
        public static final LinearAcceleration GRAV = MetersPerSecondPerSecond.of(9.81);

        // found from https://www.desmos.com/calculator/moewwoi4pa
        /** Minimum distance that we can shoot to */
        public static final Distance MIN_SHOOTING_DISTANCE = Meters.of(1.87);
        /** Maximum distance that we can shoot to */
        public static final Distance MAX_SHOOTING_DISTANCE = Meters.of(11.89);

        /** Minimum shooting velocity (of the fuel, NOT the shooter wheel) */
        public static final LinearVelocity MIN_SHOOTING_VELOCITY = MetersPerSecond.of(6.6);
        /** Maximum shooting velocity (of the fuel, NOT the shooter wheel) */
        public static final LinearVelocity MAX_SHOOTING_VELOCITY = MetersPerSecond.of(13.6);

        // Coefficients for cubic distance formula https://www.desmos.com/calculator/hxakyltti5
        public static final double DISTANCE_A = 0.000332779;
        public static final double DISTANCE_B = -0.0262643;
        public static final double DISTANCE_C = 1.00448;
        public static final double DISTANCE_D = 4.81439;

        /** Heuristic multiplier to account for friction, slippage, etc */
        public static final double OFFSET_MULTIPLIER = 1.2;

        /** Height of the Hub */
        public static final Distance HUB_HEIGHT = Feet.of(6);
    }

    /** Constants for climb */
    public static final class ClimbConstants {
        /** CAN ID for Climb motor 1 */
        public static final int CLIMB_MOTOR_1 = 31;
        /** CAN ID for Climb motor 2 */
        public static final int CLIMB_MOTOR_2 = 32;

        /** How many revolutions to completely retract the climb */
        public static final Angle CLIMB_POSITION = Revolutions.of(1.4);

        /** Gearbox ratio */
        public static final double ROTOR_TO_MECHANISM_RATIO = 45;

        /* Motion Magic Config */
        public static final double CRUISE_SPEED = 1;
        public static final double ACCELERATION = 2;

        /** Gains used for Motion Magic slot 0. */
        @SuppressWarnings("HungarianNotationConstants")
        public static final class Slot0Gains {
            public static final double kG = 0;
            public static final double kS = 0;
            public static final double kV = 0;
            public static final double kA = 0;
            public static final double kP = 32;
            public static final double kI = 0;
            public static final double kD = 0;
        }

        public static final Angle TOLERANCE = Revolutions.of(0.05);
    }

    /** Constants for positions of important points on the field */
    public static final class Positions {
        /** Position of half the field (y-axis) */
        public static final double HALF_FIELD_Y = 4;

        /** Position of the Blue side Hub */
        public static final Pose2d BLUE_HUB = new Pose2d(new Translation2d(4.5974, 4.034536), new Rotation2d());
        /** Position of the Red side Hub */
        public static final Pose2d RED_HUB = new Pose2d(new Translation2d(11.938, 4.034536), new Rotation2d());

        /** Position of the Blue left side (from driver view) Ferry target */
        public static final Pose2d BLUE_TOP_FERRY = new Pose2d(new Translation2d(2, 7.4), new Rotation2d());
        /** Position of the Blue right side (from driver view) Ferry target */
        public static final Pose2d BLUE_BOTTOM_FERRY = new Pose2d(new Translation2d(2, 0.6), new Rotation2d());
        /** Position of the Red left side (from driver view) Ferry target */
        public static final Pose2d RED_TOP_FERRY = new Pose2d(new Translation2d(14.5, 7.4), new Rotation2d());
        /** Position of the Red right side (from driver view) Ferry target */
        public static final Pose2d RED_BOTTOM_FERRY = new Pose2d(new Translation2d(14.5, 0.6), new Rotation2d());

    }

    /**
     * Constants for auto swerve angle alignment
     * {@link frc.robot.swerve.LookAtPositionCommand}
     */
    public static final class AutoAngleConstants {
        public static final double P = 4.75;
        public static final double I = 0;
        public static final double D = 0;

        public static final Angle TOLERANCE = Degrees.of(3);
    }
}