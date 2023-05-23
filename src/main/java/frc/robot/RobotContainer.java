// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmGoalGroup;
import frc.robot.commands.ArmToGoalCommand;
import frc.robot.commands.ArmTrajectoryCommandFactory;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DriveLockedRotCommand;
import frc.robot.util.CircularLimit;
import frc.team5431.titan.core.joysticks.CommandXboxController;
import frc.team5431.titan.core.leds.BlinkinPattern;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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
    // private final Buttonboard operator = new Buttonboard(3, 7, 3);
    private final AutonLoader autonLoader;
    private final Field2d armField = new Field2d();
    private final CircularLimit armLimit = new CircularLimit(ArmContainer.solver.getTotalLength());

    // private final SendableChooser<CommandBase> balanceStrategy = new SendableChooser<>();

    public RobotContainer() {
        autonLoader = new AutonLoader(systems);

        driver.setDeadzone(0.15);
        // operatorJoystick.setDeadzone(0.15);

        // drivebase.setDefaultCommand(new DefaultDriveCommand(
        //         systems,
        //         () -> modifyAxis(-driver.getLeftY()) * Drivebase.MAX_VELOCITY_METERS_PER_SECOND,
        //         () -> modifyAxis(-driver.getLeftX()) * Drivebase.MAX_VELOCITY_METERS_PER_SECOND,
        //         () -> modifyAxis(-driver.getRightX()) * Drivebase.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));

        drivebase.setDefaultCommand(new DefaultDriveCommand(
            systems,
            () -> {
                double inX = -driver.getLeftY(); // swap intended
                double inY = -driver.getLeftX();
                double mag = Math.hypot(inX, inY);
                double theta = Math.atan2(inY, inX);
                return Pair.of(modifyAxis(mag) * Drivebase.MAX_VELOCITY_METERS_PER_SECOND, theta);
            },
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

        // Robot.periodics.add(Pair.of(() -> {
        //     SmartDashboard.putNumber("pressure", systems.getCompressor().getPressure());
        //     SmartDashboard.putBoolean("pressure switch val", systems.getCompressor().getPressureSwitchValue());
        // }, 0.3));

        List<WPI_TalonFX> falcons = systems.getDrivebase().getMotors();
        List<CANSparkMax> sparks = new ArrayList<>(systems.getArm().getSparks());
        sparks.add(systems.getManipulator().getMotor());

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
            "Wrist",
            "Intake"
        };

        Robot.periodics.add(Pair.of(() -> {
            for (int i = 0; i < falcons.size(); i++) {
                SmartDashboard.putNumber(falconNames[i] + " Temp", falcons.get(i).getTemperature());
            }
            for (int i = 0; i < sparks.size(); i++) {
                SmartDashboard.putNumber(sparkNames[i] + " Temp", sparks.get(i).getMotorTemperature());
            }
        }, 0.2));

        // ShuffleboardTab tabBB = Shuffleboard.getTab("ButtonBoard Debug");
        // operator.iterate().forEach(t -> {
        //     tabBB.addBoolean(t.row().getLetter() + t.column(), t.trigger()::getAsBoolean)
        //     .withSize(1,1)
        //     .withPosition(t.column(), t.row().getIndex());
        // });

        // balanceStrategy.setDefaultOption("pid", new Autobalancer(systems));
        // balanceStrategy.addOption("bangbang", new AutobalancerBangBang(systems));
        // balanceStrategy.addOption("bb 3015", new AutobalancerHardcode(systems));
        // balanceStrategy.addOption("bb 3015 pid", new AutobalancerHardcodePID(systems));

        // SmartDashboard.putData("Bal Strat", balanceStrategy);
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
        driver.back().onTrue(runOnce(drivebase::zeroGyroscope));

        // D-Pad cardinal directions
        driver.povUp().whileTrue(run(
                () -> drivebase.drive(new ChassisSpeeds(Drivebase.MAX_VELOCITY_METERS_PER_SECOND*0.15, 0, 0)), drivebase));
        driver.povDown().whileTrue(run(
                () -> drivebase.drive(new ChassisSpeeds(-Drivebase.MAX_VELOCITY_METERS_PER_SECOND*0.15, 0, 0)), drivebase));
        driver.povLeft().whileTrue(run(
                () -> drivebase.drive(new ChassisSpeeds(0, Drivebase.MAX_VELOCITY_METERS_PER_SECOND*0.15, 0)), drivebase));
        driver.povRight().whileTrue(run(
                () -> drivebase.drive(new ChassisSpeeds(0, -Drivebase.MAX_VELOCITY_METERS_PER_SECOND*0.15, 0)), drivebase));


        Supplier<Pair<Double, Double>> defaultDrive = () -> {
            double inX = -driver.getLeftY(); // swap intended
            double inY = -driver.getLeftX();
            double mag = Math.hypot(inX, inY);
            double theta = Math.atan2(inY, inX);
            return Pair.of(modifyAxis(mag) * Drivebase.MAX_VELOCITY_METERS_PER_SECOND, theta);
        };

        BooleanSupplier isManualAdjustment = () -> {
            return modifyAxis(-driver.getRightX()) != 0;
        };

        driver.a().onTrue(new DriveLockedRotCommand(systems, defaultDrive, 180, isManualAdjustment));
        driver.b().onTrue(new DriveLockedRotCommand(systems, defaultDrive, 270, isManualAdjustment));
        driver.x().onTrue(new DriveLockedRotCommand(systems, defaultDrive, 90, isManualAdjustment));
        driver.y().onTrue(new DriveLockedRotCommand(systems, defaultDrive, 0, isManualAdjustment));
        // driver.leftBumper().onTrue(runOnce(() -> systems.getManipulator().setPositive()));
        // driver.rightBumper().onTrue(systems.getManipulator().manipRunCommand(GamePiece.CONE, true));

        operatorJoystick.a().toggleOnTrue(systems.getManipulator().manipRunCommand(GamePiece.CONE, true));
        operatorJoystick.b().toggleOnTrue(systems.getManipulator().manipRunCommand(GamePiece.CUBE, true));
        // operatorJoystick.y().onTrue(runOnce(ArmContainer.solver::toggleTopDefault));

        // operator.A5().or(operatorJoystick.back()).onTrue(systems.getLeds().ledRunCommand(BlinkinPattern.YELLOW)
        //     .withTimeout(8));
        // operatorJoystick.start().onTrue(systems.getLeds().ledRunCommand(BlinkinPattern.VIOLET)
        //     // .andThen(waitSeconds(8)));
        //     .withTimeout(8));
        // operator.A7().onTrue(systems.getLeds().ledRunCommand(getPatternFromAlliance())
        //     .withTimeout(5));
        // operator.B7().onTrue(systems.getLeds().ledRunCommand(getPatternFromAlliance(true))
        //     .withTimeout(5));
        // operator.C7().toggleOnTrue(systems.getLeds().ledCommand(BlinkinPattern.BLACK).andThen(waitSeconds(150)));

        // driver.back().onTrue(systems.getLeds().ledRunCommand(getPatternFromAlliance(true))
        //     .withTimeout(5));
        // driver.start().toggleOnTrue(systems.getLeds().ledCommand(BlinkinPattern.BLACK).andThen(waitSeconds(150)));

        operatorJoystick.back().onTrue(ArmTrajectoryCommandFactory.procure(systems, Constants.ARM_TRAJECTORY_CONFIG_SLOW, Constants.armBackwardsGroundCube));
        operatorJoystick.start().onTrue(ArmTrajectoryCommandFactory.procure(systems, Constants.ARM_TRAJECTORY_CONFIG_SLOW, Constants.armLowCube));
        operatorJoystick.y().onTrue(ArmTrajectoryCommandFactory.procure(systems, Constants.ARM_TRAJECTORY_CONFIG, Constants.armHighIntermediate, Constants.armHighCone));
        // operatorJoystick.back().onTrue(new ProxyCommand(balanceStrategy::getSelected));
        // operatorJoystick.back().onTrue(autonLoader.placeHighNoDrive().andThen(new ArmToGoalCommand(
        //     systems,
        //     Constants.armStow,
        //     ArmToGoalCommand.USE_INCHES)));


        operatorJoystick.povRight().onTrue(new SequentialCommandGroup( // Assisted high
            new ArmToGoalCommand(
                systems,
                Constants.armMidCone,
                ArmToGoalCommand.USE_INCHES
            ).withTimeout(0.75),
            new ArmToGoalCommand(
                systems,
                Constants.armHighCone,
                ArmToGoalCommand.USE_INCHES
            ).withTimeout(1)
        ));

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

        operatorJoystick.povUp().whileTrue(
            run(() -> systems.getArm().getWrist().add(2))
        );

        operatorJoystick.povDown().whileTrue(
            run(() -> systems.getArm().getWrist().add(-2))
        );

        operatorJoystick.leftBumper().onTrue(new ArmGoalGroup(
            systems,
            Constants.armMidCube,
            ArmToGoalCommand.USE_INCHES | ArmToGoalCommand.FINISH_INSTANTLY
        ));

        operatorJoystick.rightBumper().onTrue(new ArmGoalGroup(
            systems,
            Constants.armGroundCube,
            ArmToGoalCommand.USE_INCHES | ArmToGoalCommand.FINISH_INSTANTLY
        ));
        
        operatorJoystick.leftTrigger().onTrue(new ArmGoalGroup( // Normal Grab
            systems,
            Constants.armGroundTippedCone,
            ArmToGoalCommand.USE_INCHES | ArmToGoalCommand.FINISH_INSTANTLY
        ));

        operatorJoystick.rightTrigger().onTrue(new ArmGoalGroup(
            systems,
            Constants.armGroundUprightCone,
            ArmToGoalCommand.USE_INCHES | ArmToGoalCommand.FINISH_INSTANTLY
        ));

        operatorJoystick.x().onTrue(new SequentialCommandGroup( // Assisted high
            new ArmToGoalCommand(
                systems,
                Constants.armMidCone,
                ArmToGoalCommand.USE_INCHES
            ).withTimeout(0.75),
            new ArmToGoalCommand(
                systems,
                Constants.armHighCube,
                ArmToGoalCommand.USE_INCHES
            ).withTimeout(1)
        ));

        operatorJoystick.povLeft().onTrue(new ArmGoalGroup( // Middle node
            systems,
            Constants.armMidCone,
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
        value = deadband(value, 0.15);

        // More sensitive at smaller speeds
        double newValue = Math.pow(value, 2);

        // Copy the sign to the new value
        newValue = Math.copySign(newValue, value);

        return newValue;
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
        SmartDashboard.putData(systems.getArm().getOuter());
        armField.setRobotPose(new Pose2d(systems.getArm().getGoal().plus(new Translation2d(4.5, 2.2)), systems.getArm().getWrist().getPositionRot2d()));
        SmartDashboard.putData("Field", armField);
        systems.getArm().debugPeriodic();
    }

    public void teleopInit() {
        systems.getArm().getOuter().add(0);
        systems.getArm().getInner().add(0);
        systems.getArm().getWrist().add(0);

        systems.getArm().setGoalToCurrentPosition();
    }

    public void disabledInit() {
        operatorJoystick.getHID().setRumble(RumbleType.kLeftRumble, 0.0);
    }
}
