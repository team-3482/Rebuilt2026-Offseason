// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.CalculationConstants;
import frc.robot.constants.Constants.RobotConstants;
import frc.robot.constants.Constants.ShooterConstants;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

/** Controls the Shooter */
public class ShooterSubsystem extends SubsystemBase {
    // Use Bill Pugh Singleton Pattern for efficient lazy initialization (thread-safe !)
    private static class ShooterSubsystemHolder {
        private static final ShooterSubsystem INSTANCE = new ShooterSubsystem();
    }

    /** Always use this method to get the singleton instance of this subsystem. */
    public static ShooterSubsystem getInstance() {
        return ShooterSubsystemHolder.INSTANCE;
    }

    @SuppressWarnings("FieldCanBeLocal")
    private final TalonFX shooterMotor1 = new TalonFX(ShooterConstants.SHOOTER_MOTOR_1, RobotConstants.CAN_BUS);
    @SuppressWarnings("FieldCanBeLocal")
    private final TalonFX shooterMotor2 = new TalonFX(ShooterConstants.SHOOTER_MOTOR_2, RobotConstants.CAN_BUS);
    private final TalonFX shooterMotor3 = new TalonFX(ShooterConstants.SHOOTER_MOTOR_3, RobotConstants.CAN_BUS);
    private final TalonFX feederMotor = new TalonFX(ShooterConstants.FEEDER_MOTOR, RobotConstants.CAN_BUS);
    private final TalonFX sterilizerMotor = new TalonFX(ShooterConstants.STERILIZER_MOTOR, RobotConstants.CAN_BUS);

    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0);

    private AngularVelocity lastTargetVelocity = RPM.of(1);

    private ShooterSubsystem() {
        super("ShooterSubsystem");

        Follower follower = new Follower(ShooterConstants.SHOOTER_MOTOR_3, MotorAlignmentValue.Opposed);
        shooterMotor1.setControl(follower);
        shooterMotor2.setControl(follower);

        this.configureMotors();

    }

    @Override
    public void periodic() {
        Logger.recordOutput("Shooter/ShooterVelocity", getShooterVelocity().in(RPM));
    }

    /**
     * A helper method that configures MotionMagic on both motors.
     */
    private void configureMotors() {
        TalonFXConfiguration configuration = new TalonFXConfiguration();

        MotorOutputConfigs motorOutputConfigs = configuration.MotorOutput;
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;

        FeedbackConfigs feedbackConfigs = configuration.Feedback;
        feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        feedbackConfigs.SensorToMechanismRatio = ShooterConstants.GEAR_RATIO;

        // Set Motion Magic gains in slot 0.
        Slot0Configs slot0Configs = configuration.Slot0;
        slot0Configs.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
        slot0Configs.kS = ShooterConstants.Slot0Gains.kS;
        slot0Configs.kV = ShooterConstants.Slot0Gains.kV;
        slot0Configs.kP = ShooterConstants.Slot0Gains.kP;
        slot0Configs.kI = ShooterConstants.Slot0Gains.kI;
        slot0Configs.kD = ShooterConstants.Slot0Gains.kD;

        this.shooterMotor3.getConfigurator().apply(configuration);
    }

    /**
     * Sets the angular velocity of the shooter motors with PID
     * @param targetAngularVelocity the target angular velocity for the shooter motors.
     */
    public void setShooterAngularVelocity(AngularVelocity targetAngularVelocity) {
        lastTargetVelocity = targetAngularVelocity;
        shooterMotor3.setControl(velocityVoltage.withVelocity(targetAngularVelocity));
    }

    /**
     * Set the speed of the shooter motors
     * @param speed the speed from -1 to 1
     */
    public void setShooterSpeed(double speed) {
        shooterMotor3.set(speed);
    }

    /**
     * Set the speed of the feeder motor
     * @param speed the speed from -1 to 1
     */
    public void setFeederSpeed(double speed) {
        feederMotor.set(speed);
    }

    /**
     * Set the speed of the sterilizer motor
     * @param speed the speed from -1 to 1
     */
    public void setSterilizerSpeed(double speed) {
        sterilizerMotor.set(speed);
    }

    /**
     * Get the velocity of the shooter motors
     * @return the velocity
     */
    public AngularVelocity getShooterVelocity() {
        return shooterMotor3.getVelocity().getValue();
    }

    /**
     * Check for if shooter motors are revved up to shooting velocity
     * @return true if shooter velocity is within threshold
     */
    public boolean isShooterVelocityWithinTolerance() {
        double difference = (getShooterVelocity().in(RPM) - lastTargetVelocity.in(RPM));
        return Math.abs(difference) <= ShooterConstants.VELOCITY_TOLERANCE.in(RPM);
    }

    /**
     * Calculate the desired fuel velocity based on distance from the target
     * @param distance distance from the target
     * @return the desired velocity of the fuel
     */
    private LinearVelocity calculateFuelLinearVelocity(Distance distance) {
        double d = distance.in(Meters);
        double v = MathUtil.clamp(
            (
                CalculationConstants.DISTANCE_A * Math.pow(d, 3)
                + CalculationConstants.DISTANCE_B * Math.pow(d, 2)
                + CalculationConstants.DISTANCE_C * d
                + CalculationConstants.DISTANCE_D
            ),
            CalculationConstants.MIN_SHOOTING_VELOCITY.in(MetersPerSecond),
            CalculationConstants.MAX_SHOOTING_VELOCITY.in(MetersPerSecond)
        );

        return MetersPerSecond.of(v);
    }

    /**
     * Calculate the desired fuel velocity based on distance from the target
     * @param distance distance from the target
     * @param angle the angle of the shooter
     * @return the desired velocity of the fuel
     */
    public static double calculateExitVelocity(
            Distance distanceX,
            Angle angle) {

        double angleInRadians = angle.in(Radians);
        double xInMeters = distanceX.in(Meters);

        double denominator =
                2.0
                * Math.pow(Math.cos(angleInRadians), 2)
                * (xInMeters * Math.tan(angleInRadians) 
                - (CalculationConstants.HUB_HEIGHT.in(Meters) - CalculationConstants.RELEASE_HEIGHT.in(Meters)));

        if (denominator <= 0) {
            throw new IllegalArgumentException(
            "Target cannot be reached with this angle and position."
            );
        }

        return Math.sqrt((CalculationConstants.GRAV.in(MetersPerSecondPerSecond) * xInMeters * xInMeters) / denominator);
    }

    /**
     * Calculate the desired fuel velocity based on distance from the target
     * @param distance distance from the target
     * @return the desired velocity of the fuel
     */
    public static double calculateExitVelocity(Distance distance) {
        return calculateExitVelocity(distance, CalculationConstants.SHOOTER_ANGLE);
    }


    /**
     * Returns the target angular velocity for the shooter motors given the distance from a target.
     * @param distance the distance from the target.
     * @return The desired/target shooter angular velocity.
     */
    // public AngularVelocity calculateShooterAngularVelocity(Distance distance) {
    //     double linearVelocity = calculateFuelLinearVelocity(distance).in(MetersPerSecond);
    //     Logger.recordOutput("Shooter/FuelLinearVelocity", linearVelocity);

    //     return RadiansPerSecond.of(
    //         (linearVelocity * 2)
    //         / CalculationConstants.WHEEL_DIAMETER.in(Meters)
    //     );
    // }

    /**
     * Returns the target angular velocity for the shooter motors given the distance from a target.
     * @param distance the distance from the target.
     * @return The desired/target shooter angular velocity.
     */
    public AngularVelocity calculateShooterAngularVelocity(Distance distance) {
        double linearVelocity = calculateFuelLinearVelocity(distance).in(MetersPerSecond);
        Logger.recordOutput("Shooter/FuelLinearVelocity", linearVelocity);

        return RadiansPerSecond.of(
            (linearVelocity * CalculationConstants.FUEL_LINEAR_TO_SHOOTER_ANGULAR_VELOCITY_RATIO)
            * CalculationConstants.OFFSET_MULTIPLIER
        );
    }

    /**
     * Calculate the Linear Velocity of the shot fuel using a planetary gearbox formula.
     * @return The Linear Velocity.
     */
    public LinearVelocity getFuelLinearVelocity(Distance distance) {
        AngularVelocity currentVelocity = getShooterVelocity();
        // use target number by default, but use real velocity once revved so it can correct the angle if necessary
        AngularVelocity angularVelocity = (isShooterVelocityWithinTolerance() && currentVelocity.in(RotationsPerSecond) > 1)
            ? currentVelocity
            : calculateShooterAngularVelocity(distance);

        return MetersPerSecond.of(
            angularVelocity.in(RadiansPerSecond) * CalculationConstants.SHOOTER_ANGULAR_TO_FUEL_LINEAR_VELOCITY_RATIO
        );
    }

    /**
     * Set the lastTargetVelocity to a value
     * @param velocity The new target velocity.
     */
    public void setLastTargetVelocity(AngularVelocity velocity) {
        this.lastTargetVelocity = velocity;
    }
}