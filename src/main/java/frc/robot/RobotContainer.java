// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.ArmMoveCommandGroup;
import frc.robot.commands.ArmToGoalCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.*;
import frc.robot.util.CircularLimit;
import frc.robot.util.KinematicsSolver;
import frc.team5431.titan.core.joysticks.CommandXboxController;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.HashMap;
import java.util.List;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
    private final Systems systems = new Systems();
    private final Drivebase drivebase = systems.getDrivebase();
    
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);
    private final CircularLimit armLimit = new CircularLimit(Units.inchesToMeters(34) + Units.inchesToMeters(26));
    
    private Command autonCommand;

    public RobotContainer() {
        driver.setDeadzone(0.15);
        operator.setDeadzone(0.15);

        drivebase.setDefaultCommand(new DefaultDriveCommand(
            systems,
            () -> modifyAxis(-driver.getLeftY()) * Drivebase.MAX_VELOCITY_METERS_PER_SECOND,
            () -> modifyAxis(-driver.getLeftX()) * Drivebase.MAX_VELOCITY_METERS_PER_SECOND,
            () -> modifyAxis(-driver.getRightX()) * Drivebase.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
        ));

        // systems.getArm().setDefaultCommand(systems.getArm().defaultCommand(
        //     () -> modifyAxis(driver.getRightTriggerAxis() - driver.getLeftTriggerAxis()),
        //     () -> modifyAxis(driver.getRightY())
        //     // () -> {
        //     //     double power = 0.0;
        //     //     if (driver.rightBumper().getAsBoolean())
        //     //         power += 1;
        //     //     if (driver.leftBumper().getAsBoolean())
        //     //         power -= 1;
        //     //     return power;
        //     // }
        // ));

        // CommandScheduler.getInstance().removeDefaultCommand(systems.getArm());

        configureBindings();
        initAutoPaths();

        Robot.periodics.add(Pair.of(() -> {
            SmartDashboard.putNumber("pressure", systems.getCompressor().getPressure());
            SmartDashboard.putBoolean("pressure switch val", systems.getCompressor().getPressureSwitchValue());
        }, 0.3));

        List<WPI_TalonFX> falcons = systems.getDrivebase().getMotors();
        List<CANSparkMax> sparks = systems.getArm().getSparks();

        String[] falconNames = new String[]{
            "FLSteer",
            "FLDrive",
            "FRSteer",
            "FRDrive",
            "BLSteer",
            "BLDrive",
            "BRSteer",
            "BRDrive"
        };

        String[] sparkNames = new String[]{
            "OuterL",
            "OuterR",
            "InnerL",
            "InnerR",
            "Wrist"
        };

        Robot.periodics.add(Pair.of(() -> {
            for (int i = 0; i < falcons.size(); i++) {
                SmartDashboard.putNumber(falconNames[i] + " Temp", falcons.get(i).getTemperature());
            }
            for (int i = 0; i < sparks.size(); i++) {
                SmartDashboard.putNumber(sparkNames[i] + " Temp", sparks.get(i).getMotorTemperature());
            }
        }, 0.1));

        //systems.getArm().setDefaultCommand(autonCommand);
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

        driver.leftBumper().onTrue(runOnce(() -> systems.getManipulator().open()));
        driver.rightBumper().onTrue(runOnce(() -> systems.getManipulator().close()));
        driver.x().onTrue(runOnce(() -> systems.getDeadwheels().toggle()));
        // This controlls the arm, however we decided this would be on the driver's side. Not the operator's. - Rudy (and josh & brain)
        driver.rightTrigger().onTrue(new ArmMoveCommandGroup(
            systems,
            new Translation2d(14.34, -11.95),
            ArmToGoalCommand.FINISH_INSTANTLY | ArmToGoalCommand.USE_INCHES,
            302,
            false
        ));


        // operator.y().toggleOnTrue(systems.getIntake().floorIntakeCommand());
        // operator.a().onTrue(systems.getIntake().intakeStow());
        // operator.b().onTrue(runOnce(() -> systems.getIntake().toggle()));
        // operator.x().whileTrue(systems.getIntake().runIntakeCommand(false));

        // operator.rightBumper().onTrue(new JumpToGoalPositionCommand( // Start
        //     systems.getArm(),
        //     new Translation2d(4.38, -29.34),
        //     JumpToGoalPositionCommand.FINISH_INSTANTLY | JumpToGoalPositionCommand.USE_INCHES
        // ).alongWith(
        //     new WristAngleCommand(systems.getArm().getWrist(), 104)
        // ).alongWith(
        //     new WristOpenCommand(systems.getManipulator(), true)
        // ));
        operator.rightBumper().onTrue(new ArmMoveCommandGroup(
            systems,
            new Translation2d(4, -25),
            ArmToGoalCommand.FINISH_INSTANTLY | ArmToGoalCommand.USE_INCHES,
            0,
            false
        ));

        systems.getArm().getCurrentCommand();

        // operator.leftTrigger().onTrue(new JumpToGoalPositionCommand( // Normal Grab
        //     systems.getArm(),
        //     new Translation2d(6.17, -34.24),
        //     JumpToGoalPositionCommand.FINISH_INSTANTLY | JumpToGoalPositionCommand.USE_INCHES
        // ).alongWith(
        //     new WristAngleCommand(systems.getArm().getWrist(), 0)
        // ).alongWith(
        //     new WristOpenCommand(systems.getManipulator(), true)
        // ));
        operator.leftTrigger().onTrue(new ArmMoveCommandGroup(
            systems,
            new Translation2d(6.17, -34.24),
            ArmToGoalCommand.FINISH_INSTANTLY | ArmToGoalCommand.USE_INCHES,
            280,
            true
        ));

        // operator.rightTrigger().onTrue(new JumpToGoalPositionCommand( // Inverted Grab
        //     systems.getArm(),
        //     new Translation2d(3.84, -25.69),
        //     JumpToGoalPositionCommand.FINISH_INSTANTLY | JumpToGoalPositionCommand.USE_INCHES
        // ).alongWith(
        //     new WristAngleCommand(systems.getArm().getWrist(), 280)
        // ));

        operator.rightTrigger().onTrue(new ArmMoveCommandGroup(
            systems,
            new Translation2d(3.84, -25.69),
            ArmToGoalCommand.FINISH_INSTANTLY | ArmToGoalCommand.USE_INCHES,
            280,
            true
        ));

        // In  theory, the top IK possibility would be more optimal for this node. However we cant set the possibility without problems
        operator.povRight().onTrue( // High node
            systems.getArm().getWrist().setDegreesCommand(0)
        .andThen(new ArmToGoalCommand(
            systems,
            new Translation2d(40.875, 21.69),
            ArmToGoalCommand.USE_INCHES | ArmToGoalCommand.FINISH_INSTANTLY
        ).alongWith(new RunCommand(() -> {
                Arm.solver.preferTopByDefault = false;
        }))));

        operator.povLeft().onTrue(new ArmToGoalCommand( // Middle node & grab from slidy boi
            systems,
            new Translation2d(32.03, 2.83),
            ArmToGoalCommand.FINISH_INSTANTLY | ArmToGoalCommand.USE_INCHES
        ));

        // operator.leftBumper().onTrue(runOnce(() -> systems.getArm().incrOut(-10)));
        // operator.rightBumper().onTrue(runOnce(() -> systems.getArm().incrOut(10)));
        // operator.back().onTrue(runOnce(() -> systems.getArm().incrIn(-10))); // elbow runs opposite dir
        // operator.start().onTrue(runOnce(() -> systems.getArm().incrIn(10)));
        operator.povDown().onTrue(runOnce(() -> systems.getArm().getWrist().add(-20)));
        operator.povUp().onTrue(runOnce(() -> systems.getArm().getWrist().add(20)));

        operator.rightStick().onTrue(new InstantCommand(() -> {Arm.solver.preferTopByDefault = !Arm.solver.preferTopByDefault;}));
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
        // TODO extract into default command for arm
        var gp = systems.getArm().getGoal();
        double leftx = operator.getLeftX();
        double lefty = -operator.getLeftY();
        if (RobotBase.isSimulation()) {
            lefty = -lefty;
            leftx = -leftx;
        }
        if (Math.abs(leftx) < 0.15) leftx = 0;
        if (Math.abs(lefty) < 0.15) lefty = 0;
        leftx *= 0.01;
        lefty *= 0.01;
        double tx = gp.getX() + leftx;
        double ty = gp.getY() + lefty;


        gp = new Translation2d(tx, ty);
        
        if(!armLimit.isPointInsideCircle(gp)) {
            gp = armLimit.getClosestPointOnCircle(gp);
        }

        gp = new Translation2d(
            MathUtil.clamp(gp.getX(), -Units.inchesToMeters(26 + 48 - 13.125), Units.inchesToMeters(6 + 48  - 13.125)), 
            MathUtil.clamp(gp.getY(), -Units.inchesToMeters(39 + 1.5), -Units.inchesToMeters(39 + 1.5 - 78))
        );
        
        systems.getArm().setGoal(gp);

        SmartDashboard.putData(CommandScheduler.getInstance());
    }

    public void robotPeriodic() {
    }

    public void teleopInit() {
        systems.getArm().getOuter().add(0);
        systems.getArm().getInner().add(0);
        systems.getArm().getWrist().add(0);

        systems.getArm().setGoalToCurrentPosition();
    }
}
