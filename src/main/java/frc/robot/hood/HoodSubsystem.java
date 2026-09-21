// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.CalculationConstants;
import frc.robot.constants.Constants.HoodConstants;

import static edu.wpi.first.units.Units.*;

// Servo code from https://github.com/wcpllc/2026CompetitiveConcept/

/** Controls the Shooter Hood */
public class HoodSubsystem extends SubsystemBase {
    // Use Bill Pugh Singleton Pattern for efficient lazy initialization (thread-safe !)
    private static class HoodSubsystemHolder {
        private static final HoodSubsystem INSTANCE = new HoodSubsystem();
    }

    /** Always use this method to get the singleton instance of this subsystem. */
    public static HoodSubsystem getInstance() {
        return HoodSubsystemHolder.INSTANCE;
    }

    private final Servo servo1 = new Servo(HoodConstants.HOOD_SERVO_1);
    private final Servo servo2 = new Servo(HoodConstants.HOOD_SERVO_2);

    private double currentPosition = 0;
    private double targetPosition = 0;
    private Time lastUpdateTime = Seconds.of(0);

    private HoodSubsystem() {
        super("HoodSubsystem");

        servo1.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
        servo2.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
        setHoodPosition(currentPosition);
    }

    @Override
    public void periodic() {
        final Time currentTime = Seconds.of(Timer.getFPGATimestamp());
        final Time elapsedTime = currentTime.minus(lastUpdateTime);
        lastUpdateTime = currentTime;

        boolean withinTolerance = isPositionWithinTolerance();
        if (withinTolerance) {
            currentPosition = targetPosition;
        }

        final Distance maxDistanceTraveled = HoodConstants.MAX_SERVO_SPEED.times(elapsedTime);
        final double maxPercentageTraveled = maxDistanceTraveled.div(HoodConstants.SERVO_LENGTH).in(Value);
        currentPosition = targetPosition > currentPosition
            ? Math.min(targetPosition, currentPosition + maxPercentageTraveled)
            : Math.max(targetPosition, currentPosition - maxPercentageTraveled);

        // Logger.recordOutput("Hood/Position", currentPosition);

        // Logger.recordOutput("Hood/WithinTolerance", withinTolerance);
        // SmartDashboard.putBoolean("Hood/WithinTolerance", withinTolerance);
    }

    /**
     * Convert the angle to servo value from 0.0 to 1.0
     * @param angle within minimum and maximum set in {@link HoodConstants}
     */
    public double hoodAngleToDouble(Angle angle) {
        return 1 - (
            angle.in(Degrees) - HoodConstants.HOOD_ANGLE_MIN.in(Degrees))
            / (HoodConstants.HOOD_ANGLE_MAX.in(Degrees) - HoodConstants.HOOD_ANGLE_MIN.in(Degrees)
        );
    }

    /**
     * Set the position of the Hood
     * @param position from 0.0 to 1.0
     */
    public void setHoodPosition(double position) {
        double clampedPosition = MathUtil.clamp(position, 0, 1);
        if (!Double.isNaN(clampedPosition)) {
            servo1.set(clampedPosition);
            servo2.set(clampedPosition);
            targetPosition = clampedPosition;
        } else {
            System.out.println("!!! Hood position change failed: target position is NaN !!!");
            //Elastic.sendNotification(new Notification(NotificationLevel.ERROR, "Hood Subsystem", "Position move failed: target position is NaN!"));
        }
    }

    /**
     * Set position of Hood using an angle
     * @param angle Between Minimum and Maximum from {@link HoodConstants}
     */
    public void setHoodAngle(Angle angle) {
        setHoodPosition(hoodAngleToDouble(angle));
    }

    /**
     * Gives the angle at which the hood should be set to shoot a fuel element to
     * @param distance The distance from the bot to the target.
     * @param velocity The linear velocity that the Fuel will shoot at.
     * @param hub Whether to calculate to shoot into the hub.
     * @return The angle at which the hood should be set.
     */
    public Angle getShootingHoodAngle(Distance distance, LinearVelocity velocity, boolean hub) {
        double v = Math.abs((velocity).in(MetersPerSecond));
        double g = CalculationConstants.GRAV.in(MetersPerSecondPerSecond);
        double d = distance.in(Meters);
        double h = CalculationConstants.HUB_HEIGHT.in(Meters);

        return Radians.of(Math.atan(( // https://www.desmos.com/calculator/moewwoi4pa :)
            Math.pow(v, 2)
            + Math.sqrt(
                Math.pow(v, 4)
                - g * (
                    g * Math.pow(d, 2)
                    + (hub ? 2 * h * Math.pow(v, 2) : 0)
                )
            )) / (g * d)
        ));
    }

    /**
     * Gives the angle at which the hood should be set to shoot a fuel element to
     * @param distance The distance from the bot to the target
     * @param velocity The linear velocity that the Fuel will shoot at.
     * @return The angle at which the hood should be set.
     */
    public Angle getShootingHoodAngle(Distance distance, LinearVelocity velocity) {
        return getShootingHoodAngle(distance, velocity, true);
    }

    /**
     * Finds if Hood is actively within tolerance to target position
     * @return whether it's within tolerance
     */
    public boolean isPositionWithinTolerance() {
        return MathUtil.isNear(targetPosition, currentPosition, 0.01);
    }
}