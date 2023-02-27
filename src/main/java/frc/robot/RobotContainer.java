// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.ArmMoveCommandGroup;
import frc.robot.commands.ArmToGoalCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.util.CircularLimit;
import frc.robot.util.PresetPosition;
import frc.team5431.titan.core.joysticks.CommandXboxController;
import frc.team5431.titan.core.leds.BlinkinPattern;

import java.util.List;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivebase;

public class RobotContainer {
    private final Systems systems = new Systems();
    public final Drivebase drivebase = systems.getDrivebase();


    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);
    private final AutonLoader autonLoader;
    private final CircularLimit armLimit = new CircularLimit(Units.inchesToMeters(34) + Units.inchesToMeters(26));

    public RobotContainer() {

        driver.setDeadzone(0.15);
        operator.setDeadzone(0.15);

        drivebase.setDefaultCommand(new DefaultDriveCommand(
                systems,
                () -> modifyAxis(-driver.getLeftY()) * Drivebase.MAX_VELOCITY_METERS_PER_SECOND,
                () -> modifyAxis(-driver.getLeftX()) * Drivebase.MAX_VELOCITY_METERS_PER_SECOND,
                () -> modifyAxis(-driver.getRightX()) * Drivebase.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));

        systems.getLeds().setDefaultCommand(runEnd(
            () -> systems.getLeds().ledCommand(getPatternFromAlliance()), 
            () -> {}, 
            systems.getLeds()));

        // systems.getArm().setDefaultCommand(systems.getArm().defaultCommand(
        // () -> modifyAxis(driver.getRightTriggerAxis() - driver.getLeftTriggerAxis()),
        // () -> modifyAxis(driver.getRightY())
        // // () -> {
        // // double power = 0.0;
        // // if (driver.rightBumper().getAsBoolean())
        // // power += 1;
        // // if (driver.leftBumper().getAsBoolean())
        // // power -= 1;
        // // return power;
        // // }
        // ));

        // CommandScheduler.getInstance().removeDefaultCommand(systems.getArm());

        configureBindings();

        Robot.periodics.add(Pair.of(() -> {
            SmartDashboard.putNumber("pressure", systems.getCompressor().getPressure());
            SmartDashboard.putBoolean("pressure switch val", systems.getCompressor().getPressureSwitchValue());
        }, 0.3));

        List<WPI_TalonFX> falcons = systems.getDrivebase().getMotors();
        List<CANSparkMax> sparks = systems.getArm().getSparks();

        String[] falconNames = new String[] {
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
        autonLoader = new AutonLoader(systems);

    }

    public static BlinkinPattern getPatternFromAlliance() {
        return switch (DriverStation.getAlliance()) {
            case Blue -> BlinkinPattern.BLUE;
            case Red -> BlinkinPattern.RED;
            default -> BlinkinPattern.GREEN;
        };
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
        driver.leftTrigger().onTrue(new ArmToGoalCommand( // Stow
            systems,
            PresetPosition.fromGoal(new Translation2d(Constants.armStowX, Constants.armStowY), Constants.wristStowAngle),
            ArmToGoalCommand.USE_INCHES | ArmToGoalCommand.FINISH_INSTANTLY
        ));
        driver.rightTrigger().onTrue(new ArmMoveCommandGroup( // Arm while traveling
            systems,
            new Translation2d(14.34, -11.95),
            ArmToGoalCommand.USE_INCHES | ArmToGoalCommand.FINISH_INSTANTLY,
            302,
            false
        ));

        operator.back().onTrue(systems.getLeds().ledCommand(BlinkinPattern.YELLOW)
            .andThen(waitSeconds(5)));
        operator.start().onTrue(systems.getLeds().ledCommand(BlinkinPattern.VIOLET)
            .andThen(waitSeconds(5)));
        driver.back().onTrue(systems.getLeds().ledCommand(BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE));
        driver.start().toggleOnTrue(systems.getLeds().ledCommand(BlinkinPattern.BLACK).withTimeout(150));

        operator.y().toggleOnTrue(systems.getIntake().floorIntakeCommand());
        operator.a().onTrue(systems.getIntake().intakeStow());
        operator.b().onTrue(runOnce(() -> systems.getIntake().toggle()));
        operator.x().whileTrue(systems.getIntake().runIntakeCommand(false));
        // operator.start().whileTrue(systems.getIntake().runSameDirection(false));

        // operator.rightBumper().onTrue(new ArmMoveCommandGroup( // Start?
        //     systems,
        //     new Translation2d(4, -25), // 4.38, -29.34?
        //     ArmToGoalCommand.USE_INCHES | ArmToGoalCommand.FINISH_INSTANTLY,
        //     104, // 104?
        //     false
        // ));

        operator.leftTrigger().onTrue(new ArmMoveCommandGroup( // Normal Grab
            systems,
            new Translation2d(Constants.armGroundX, Constants.armGroundY),
            ArmToGoalCommand.USE_INCHES | ArmToGoalCommand.FINISH_INSTANTLY,
            Constants.wristGroundAngle,
            true
        ));

        operator.rightTrigger().onTrue(new ArmMoveCommandGroup( // Inverted Grab
            systems,

            new Translation2d(Constants.armInnerGrabX, Constants.armInnerGrabY),
            ArmToGoalCommand.USE_INCHES | ArmToGoalCommand.FINISH_INSTANTLY,
            Constants.wristInvertAngle,
            true
        ));

        // In  theory, the top IK possibility would be more optimal for this node. However we cant set the possibility without problems
        operator.povRight().onTrue( // High node
            systems.getArm().getWrist().setDegreesCommand(0)
        .andThen(new ArmToGoalCommand(
            systems,
            PresetPosition.fromGoal(new Translation2d(Constants.armHighX, Constants.armHighY), Constants.wristHighAngle),
            ArmToGoalCommand.USE_INCHES | ArmToGoalCommand.FINISH_INSTANTLY
        )));

        operator.povLeft().onTrue(new ArmToGoalCommand( // Middle node & grab from slidy boi
            systems,
            new Translation2d(Constants.armMidX, Constants.armMidY),
            ArmToGoalCommand.USE_INCHES | ArmToGoalCommand.FINISH_INSTANTLY
        ));


        // operator.leftBumper().onTrue(runOnce(() -> systems.getArm().incrOut(-10)));
        // operator.rightBumper().onTrue(runOnce(() -> systems.getArm().incrOut(10)));
        // operator.back().onTrue(runOnce(() -> systems.getArm().incrIn(-10))); // elbow runs opposite dir
        // operator.start().onTrue(runOnce(() -> systems.getArm().incrIn(10)));
        operator.povDown().onTrue(runOnce(() -> systems.getArm().getWrist().add(-20)));
        operator.povUp().onTrue(runOnce(() -> systems.getArm().getWrist().add(20)));

    }

    public Command getAutonomousCommand() {
        // jolly good
        return autonLoader.procureAuton();
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
