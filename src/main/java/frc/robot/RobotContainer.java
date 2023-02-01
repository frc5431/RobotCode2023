// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.team5431.titan.core.joysticks.CommandXboxController;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class RobotContainer {
    private final Systems systems = new Systems();
    private final Drivebase drivebase = systems.getDrivebase();
    
    private final CommandXboxController driver = new CommandXboxController(0);
    private Command autonCommand;

    public RobotContainer() {
        driver.setDeadzone(0.15);

        // drivebase.setDefaultCommand(new DefaultDriveCommand(
        //     systems,
        //     () -> modifyAxis(-driver.getLeftY()) * Drivebase.MAX_VELOCITY_METERS_PER_SECOND,
        //     () -> modifyAxis(-driver.getLeftX()) * Drivebase.MAX_VELOCITY_METERS_PER_SECOND,
        //     () -> modifyAxis(-driver.getRightX()) * Drivebase.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
        // ));

        systems.getArm().setDefaultCommand(systems.getArm().defaultCommand(
            () -> modifyAxis(driver.getRightTriggerAxis() - driver.getLeftTriggerAxis()),
            () -> modifyAxis(driver.getRightY())
        ));

        configureBindings();
        initAutoPaths();
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
        
        driver.a().onTrue(runOnce(() -> systems.getDblSol1().toggle()));
        driver.b().onTrue(runOnce(() -> systems.getDblSol2().toggle()));
        driver.x().onTrue(runOnce(() -> systems.getSglSol1().toggle()));

        // driver.leftBumper().whileTrue(systems.getArm().runArmCommand(0.1));
        // driver.leftBumper().onTrue(runOnce(() -> systems.getArm().set(0.25)));
        // driver.rightBumper().whileTrue(systems.getArm().runArmCommand(-0.1));
        // driver.rightBumper().onTrue(runOnce(() -> systems.getArm().set(0.75)));

        driver.back().onTrue(runOnce(() -> systems.getArm().incr(-0.05)));
        driver.start().onTrue(runOnce(() -> systems.getArm().incr(0.05)));
    }

    private void initAutoPaths() {
        // This will load the file "FullAuto.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
        // for every path in the group
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Test", new PathConstraints(4, 3));

        // This is just an example event map. It would be better to have a constant, global event map
        // in your code that will be used by all path following commands.
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("marker1", new PrintCommand("Passed marker 1"));

        // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            drivebase::getPosition, // Pose2d supplier
            drivebase::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
            drivebase.m_kinematics, // SwerveDriveKinematics
            new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
            new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
            (states) -> drivebase.driveRaw(drivebase.m_kinematics.toChassisSpeeds(states)), // Module states consumer used to output to the drive subsystem
            eventMap,
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            drivebase // The drive subsystem. Used to properly set the requirements of path following commands
        );

        autonCommand = autoBuilder.fullAuto(pathGroup);
    }

    public Command getAutonomousCommand() {
        return autonCommand;
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

    public void teleopPeriodic() {
        double power = 0.0;
        if (driver.rightBumper().getAsBoolean())
            power += 0.07;
        if (driver.leftBumper().getAsBoolean())
            power -= 0.07;
        systems.getWrist().set(power);
    }
}
