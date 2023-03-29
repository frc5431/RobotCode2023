// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.commands.ArmGoalGroup;
import frc.robot.commands.ArmToGoalCommand;
import frc.robot.commands.Autobalancer;
import frc.robot.commands.AutobalancerBangBang;
import frc.robot.commands.AutobalancerHardcode;
import frc.robot.commands.AutobalancerHardcodePID;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.util.Buttonboard;
import frc.robot.util.CircularLimit;
import frc.team5431.titan.core.joysticks.CommandXboxController;
import frc.team5431.titan.core.leds.BlinkinPattern;

import java.util.List;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ArmContainer;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Manipulator.GamePiece;
import frc.robot.subsystems.Manipulator.MotorDirection;

public class RobotContainer {
    private final Systems systems = new Systems();
    public final Drivebase drivebase = systems.getDrivebase();

    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operatorJoystick = new CommandXboxController(1);
    private final Buttonboard operator = new Buttonboard(3, 7, 3);
    private final AutonLoader autonLoader;
    private final CircularLimit armLimit = new CircularLimit(ArmContainer.solver.getTotalLength());

    private final SendableChooser<CommandBase> balanceStrategy = new SendableChooser<>();

    public RobotContainer() {

        driver.setDeadzone(0.15);
        // operatorJoystick.setDeadzone(0.15);

        drivebase.setDefaultCommand(new DefaultDriveCommand(
                systems,
                () -> modifyAxis(-driver.getLeftY()) * Drivebase.MAX_VELOCITY_METERS_PER_SECOND,
                () -> modifyAxis(-driver.getLeftX()) * Drivebase.MAX_VELOCITY_METERS_PER_SECOND,
                () -> modifyAxis(-driver.getRightX()) * Drivebase.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));

        systems.getLeds().setDefaultCommand(run(
            () -> systems.getLeds().set(getPatternFromAlliance()), 
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
        }, 0.2));
        autonLoader = new AutonLoader(systems);

        // ShuffleboardTab tabBB = Shuffleboard.getTab("ButtonBoard Debug");
        // operator.iterate().forEach(t -> {
        //     tabBB.addBoolean(t.row().getLetter() + t.column(), t.trigger()::getAsBoolean)
        //     .withSize(1,1)
        //     .withPosition(t.column(), t.row().getIndex());
        // });

        balanceStrategy.setDefaultOption("pid", new Autobalancer(systems));
        balanceStrategy.addOption("bangbang", new AutobalancerBangBang(systems));
        balanceStrategy.addOption("bb 3015", new AutobalancerHardcode(systems));
        balanceStrategy.addOption("bb 3015 pid", new AutobalancerHardcodePID(systems));

        SmartDashboard.putData("Bal Strat", balanceStrategy);
    }

    public static BlinkinPattern getPatternFromAlliance() {
        return getPatternFromAlliance(false);
    }

    public static BlinkinPattern getPatternFromAlliance(boolean charging) {
        if (!charging) {
            return BlinkinPattern.BLACK;
            // return switch (DriverStation.getAlliance()) {
            //     case Blue -> BlinkinPattern.BLUE;
            //     case Red -> BlinkinPattern.RED;
            //     default -> BlinkinPattern.GREEN;
            // };
        } else {
            return switch (DriverStation.getAlliance()) {
                case Blue -> BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE;
                case Red -> BlinkinPattern.COLOR_WAVES_LAVA_PALETTE;
                default -> BlinkinPattern.COLOR_WAVES_FOREST_PALETTE;
            };
        }
    }

    private void configureBindings() {
        // Y button zeros the gyroscope
        driver.y().onTrue(runOnce(drivebase::zeroGyroscope));

        // D-Pad cardinal directions
        driver.povUp().whileTrue(run(
                () -> drivebase.drive(new ChassisSpeeds(Drivebase.MAX_VELOCITY_METERS_PER_SECOND*0.15, 0, 0)), drivebase));
        driver.povDown().whileTrue(run(
                () -> drivebase.drive(new ChassisSpeeds(-Drivebase.MAX_VELOCITY_METERS_PER_SECOND*0.15, 0, 0)), drivebase));
        driver.povLeft().whileTrue(run(
                () -> drivebase.drive(new ChassisSpeeds(0, Drivebase.MAX_VELOCITY_METERS_PER_SECOND*0.15, 0)), drivebase));
        driver.povRight().whileTrue(run(
                () -> drivebase.drive(new ChassisSpeeds(0, -Drivebase.MAX_VELOCITY_METERS_PER_SECOND*0.15, 0)), drivebase));

        // driver.leftBumper().onTrue(runOnce(() -> systems.getManipulator().setPositive()));
        // driver.rightBumper().onTrue(systems.getManipulator().manipRunCommand(GamePiece.CONE, true));

        operatorJoystick.a().toggleOnTrue(systems.getManipulator().manipRunCommand(GamePiece.CONE, true));
        operatorJoystick.b().toggleOnTrue(systems.getManipulator().manipRunCommand(GamePiece.CUBE, true));
        operatorJoystick.y().onTrue(runOnce(ArmContainer.solver::toggleTopDefault));

        // operator.A5().or(operatorJoystick.back()).onTrue(systems.getLeds().ledRunCommand(BlinkinPattern.YELLOW)
        //     .withTimeout(8));
        operator.A6().or(operatorJoystick.start()).onTrue(systems.getLeds().ledRunCommand(BlinkinPattern.VIOLET)
            // .andThen(waitSeconds(8)));
            .withTimeout(8));
        // operator.A7().onTrue(systems.getLeds().ledRunCommand(getPatternFromAlliance())
        //     .withTimeout(5));
        // operator.B7().onTrue(systems.getLeds().ledRunCommand(getPatternFromAlliance(true))
        //     .withTimeout(5));
        // operator.C7().toggleOnTrue(systems.getLeds().ledCommand(BlinkinPattern.BLACK).andThen(waitSeconds(150)));

        driver.back().onTrue(systems.getLeds().ledRunCommand(getPatternFromAlliance(true))
            .withTimeout(5));
        // driver.start().toggleOnTrue(systems.getLeds().ledCommand(BlinkinPattern.BLACK).andThen(waitSeconds(150)));

        operatorJoystick.back().onTrue(new ProxyCommand(balanceStrategy::getSelected));

        // Arm controls, but for driver by request of Phillip
        driver.leftTrigger().onTrue(new ArmGoalGroup( // Stow
            systems,
            Constants.armStow,
            ArmToGoalCommand.USE_INCHES | ArmToGoalCommand.FINISH_INSTANTLY
        ));

        driver.rightTrigger().onTrue(new ArmGoalGroup(
            systems,
            Constants.armWhileTraveling,
            ArmToGoalCommand.USE_INCHES | ArmToGoalCommand.FINISH_INSTANTLY
        ));

        operator.A1().or(operatorJoystick.povUp()).whileTrue(
            run(() -> systems.getArm().getWrist().add(2))
        );

        operator.B1().or(operatorJoystick.povDown()).whileTrue(
            run(() -> systems.getArm().getWrist().add(-2))
        );

        operatorJoystick.leftBumper().onTrue(new ArmGoalGroup(
            systems,
            Constants.armSingleSubPickup,
            ArmToGoalCommand.USE_INCHES | ArmToGoalCommand.FINISH_INSTANTLY
        ));

        operatorJoystick.rightBumper().onTrue(new ArmGoalGroup(
            systems,
            Constants.armGroundCube,
            ArmToGoalCommand.USE_INCHES | ArmToGoalCommand.FINISH_INSTANTLY
        ));

        operatorJoystick.rightTrigger().onTrue(new ArmGoalGroup(
            systems,
            Constants.armGroundUprightCone,
            ArmToGoalCommand.USE_INCHES | ArmToGoalCommand.FINISH_INSTANTLY
        ));

        // operator.A3().or(operatorJoystick.b()).onTrue(new ArmGoalGroup( // Backwards high - requires intermediate pos!
        //     systems,
        //     Constants.armBackwardsHigh,
        //     ArmToGoalCommand.USE_INCHES | ArmToGoalCommand.FINISH_INSTANTLY
        // ));

        // (operatorJoystick.leftBumper()).onTrue(new ArmGoalGroup( // Backwards double substation
        //     systems,
        //     PresetPosition.fromGoal(Constants.armBackwardsHigh.getWristPos(), 30),
        //     ArmToGoalCommand.USE_INCHES | ArmToGoalCommand.FINISH_INSTANTLY
        // ));

        // operator.B4().or(operatorJoystick.rightBumper()).onTrue(new ArmGoalGroup( // Backwards mid
        //     systems,
        //     Constants.armBackwardsMid,
        //     ArmToGoalCommand.USE_INCHES | ArmToGoalCommand.FINISH_INSTANTLY
        // ));

        // operator.A4().or(operatorJoystick.x()).or(driver.a()).onTrue(new ArmGoalGroup( // Intermediate
        //     systems,
        //     Constants.armBackwardsIntermediate,
        //     ArmToGoalCommand.USE_INCHES | ArmToGoalCommand.FINISH_INSTANTLY
        // ));

        operator.C2().or(operatorJoystick.leftTrigger()).onTrue(new ArmGoalGroup( // Normal Grab
            systems,
            Constants.armNormalGrabOld,
            ArmToGoalCommand.USE_INCHES | ArmToGoalCommand.FINISH_INSTANTLY
        ));

        // In theory, the top IK possibility would be more optimal for this node. However we cant set the possibility without problems
        operator.A2().or(operatorJoystick.povRight()).onTrue( // High node
            systems.getArm().getWrist().setDegreesCommand(0)
        .andThen(new ArmGoalGroup(
            systems,
            Constants.armHigh,
            ArmToGoalCommand.USE_INCHES | ArmToGoalCommand.FINISH_INSTANTLY
        )));

        operator.B2().or(operatorJoystick.povLeft()).onTrue(new ArmGoalGroup( // Middle node
            systems,
            Constants.armMid,
            ArmToGoalCommand.USE_INCHES | ArmToGoalCommand.FINISH_INSTANTLY
        ));

        // operator.leftBumper().onTrue(runOnce(() -> systems.getArm().incrOut(-10)));
        // operator.rightBumper().onTrue(runOnce(() -> systems.getArm().incrOut(10)));
        // operator.back().onTrue(runOnce(() -> systems.getArm().incrIn(-10))); // elbow runs opposite dir
        // operator.start().onTrue(runOnce(() -> systems.getArm().incrIn(10)));
        // operatorJoystick.povDown().onTrue(runOnce(() -> systems.getArm().getWrist().add(-20)));
        // operatorJoystick.povUp().onTrue(runOnce(() -> systems.getArm().getWrist().add(20)));
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
        double leftx = operatorJoystick.getLeftX();
        double lefty = -operatorJoystick.getLeftY();
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
            MathUtil.clamp(gp.getX(), -Units.inchesToMeters(26 + 48 - 13.5), Units.inchesToMeters(6 + 48  - 13.5)), 
            MathUtil.clamp(gp.getY(), -Units.inchesToMeters(39 + 1.5), -Units.inchesToMeters(39 + 1.5 - 78))
        );
        
        systems.getArm().setGoal(gp);

        if (systems.getManipulator().currentDirection != MotorDirection.NONE)
            operatorJoystick.getHID().setRumble(RumbleType.kLeftRumble, 0.3);
        else
            operatorJoystick.getHID().setRumble(RumbleType.kLeftRumble, 0.0);
    }

    public void robotPeriodic() {
        SmartDashboard.putData(CommandScheduler.getInstance());
    }

    public void teleopInit() {
        systems.getArm().getOuter().add(0);
        systems.getArm().getInner().add(0);
        systems.getArm().getWrist().add(0);

        systems.getArm().setGoalToCurrentPosition();
    }
}
