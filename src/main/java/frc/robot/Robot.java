// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import java.io.File;

public class Robot extends LoggedRobot {
    private Command auton;
    // private PowerDistribution PDP = new PowerDistribution(0, ModuleType.kCTRE);

    public Robot() {
        RobotContainer.getInstance().configureDriverBindings();
        RobotContainer.getInstance().configureOperatorBindings();

        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
        Logger.recordMetadata("ProjectName", "Rebuilt2026");

        if (isReal()) {
            String event = DriverStation.getEventName();
            String path = "/U/logs/" + (event.isEmpty() ? "testing" : event);

            System.out.println("logging to: " + path + " (new directory: " + new File(path).mkdirs() + ")");

            SignalLogger.setPath(path);

            Logger.addDataReceiver(new WPILOGWriter(path)); // Log to a USB stick ("/U/logs")
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables

            Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
            // This will always be either 0 or 1, so the > sign is used to suppress the comparing identical expressions.
            // noinspection ConstantValue (IntelliJ warning suppression)
            Logger.recordMetadata("GitDirty", BuildConstants.DIRTY > 0 ? "true" : "false");
            Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);

            Logger.start();
        }

        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
        // Eager-load the auton command so it's ready right away
        RobotContainer.getInstance().getAutonomousCommand();
    }


    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        double voltage = RobotController.getBatteryVoltage();
        // double current = PDP.getTotalCurrent();

        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
        SmartDashboard.putNumber("Voltage", voltage);
        // SmartDashboard.putNumber("Current", current);

        Logger.recordOutput("Voltage", voltage);
        // Logger.recordOutput("Current", current);
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        auton = RobotContainer.getInstance().getAutonomousCommand();

        Logger.recordOutput("Auton/AutonCommand", auton.getName());

        if (auton != null) {
            CommandScheduler.getInstance().schedule(auton);
        } else {
            System.err.println("No auton command found.");
        }

        // Elastic.selectTab(DriverStationConstants.AUTON_TAB);
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (auton != null) {
            CommandScheduler.getInstance().cancel(auton);
        }

        // Elastic.selectTab(DriverStationConstants.TELEOP_TAB);
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {}

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}