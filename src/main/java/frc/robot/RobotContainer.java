// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.*;
import frc.team5431.titan.core.joysticks.CommandXboxController;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class RobotContainer {
    private final Systems systems = new Systems();
    private final Drivebase drivebase = systems.getDrivebase();
    
    private final CommandXboxController driver = new CommandXboxController(0);

    public RobotContainer() {
        drivebase.setDefaultCommand(new DefaultDriveCommand(
            systems,
            () -> modifyAxis(-driver.getLeftX()) * Drivebase.MAX_VELOCITY_METERS_PER_SECOND,
            () -> modifyAxis(driver.getLeftY()) * Drivebase.MAX_VELOCITY_METERS_PER_SECOND,
            () -> modifyAxis(-driver.getRightX()) * Drivebase.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
        ));

        configureBindings();
    }

    private void configureBindings() {
        // Y button zeros the gyroscope
        driver.y().onTrue(runOnce(drivebase::zeroGyroscope));
        
        // D-Pad cardinal directions
        driver.povUp().whileTrue(run(
                () -> drivebase.drive(new ChassisSpeeds(Drivebase.MAX_VELOCITY_METERS_PER_SECOND, 0, 0)), drivebase));
        driver.povDown().whileTrue(run(
                () -> drivebase.drive(new ChassisSpeeds(-Drivebase.MAX_VELOCITY_METERS_PER_SECOND, 0, 0)), drivebase));
        driver.povLeft().whileTrue(run(
                () -> drivebase.drive(new ChassisSpeeds(0, -Drivebase.MAX_VELOCITY_METERS_PER_SECOND, 0)), drivebase));
        driver.povRight().whileTrue(run(
                () -> drivebase.drive(new ChassisSpeeds(0, Drivebase.MAX_VELOCITY_METERS_PER_SECOND, 0)), drivebase));
        
        driver.a().onTrue(runOnce(systems.getDblSol1()::toggle));
        driver.b().onTrue(runOnce(systems.getDblSol2()::toggle));
        driver.x().onTrue(runOnce(systems.getSglSol1()::toggle));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    private static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

    private static double modifyAxis(double value) {
        // Deadband
        value = deadband(value, 0.075);

        value = value * value * value;

        // Square the axis
        // value = Math.copySign(value * value, value);

        return value;
    }
}
