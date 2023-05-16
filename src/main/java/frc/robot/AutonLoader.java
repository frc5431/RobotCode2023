package frc.robot;

import java.util.HashMap;
import java.util.List;
import java.util.Set;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.ArmToGoalCommand;
import frc.robot.commands.ArmTrajectoryCommandFactory;
import frc.robot.commands.AutobalancerHardcodePID;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Manipulator.GamePiece;

import static edu.wpi.first.wpilibj2.command.Commands.*;

public class AutonLoader extends BaseAutonLoader {

    public AutonLoader(Systems systems) {
        super(systems);
    }

    @Override
    public String[] getPaths() {
        return new String[] {
            "far", "farBalance",
            "mid", "midBalance",
            "near", "nearBalance",
            "nearTwoGPBal", "nearTwoGPHoldOne",
            "nearConeMidTwoCube",
            "nearThreeCube",
            "midTwoCubeBal",
            "nearThrowCube",
            "special", "placeHigh",
            "timedMobility",
            "timedBalance",
            "none"
        };
    }

    @Override
    public void initializeEventMap(HashMap<String, Command> eventMap) {
        eventMap.put("deadwheelDrop", none());
        eventMap.put("deadwheelRaise", none());
        eventMap.put("cubeIntake", systems.getManipulator().manipRunOnceCommand(GamePiece.CUBE, true));
        eventMap.put("cubeOuttake", systems.getManipulator().manipRunOnceCommand(GamePiece.CUBE, false));
        eventMap.put("coneIntake", systems.getManipulator().manipRunOnceCommand(GamePiece.CONE, true));
        eventMap.put("coneOuttake", systems.getManipulator().manipRunOnceCommand(GamePiece.CONE, false));
        eventMap.put("autoBalance", new AutobalancerHardcodePID(systems));
        eventMap.put("placeHighCone", placeHighCone());
        eventMap.put("placeHighCube", placeHighCube());
        eventMap.put("placeMidCone", placeMidCone());
        eventMap.put("placeLowCube", placeLowCube());
        eventMap.put("placeHighAdjacentCone", placeHighNoDrive(GamePiece.CONE));
        eventMap.put("placeHighAdjacentCube", placeHighNoDrive(GamePiece.CUBE));
        eventMap.put("stow", new ArmToGoalCommand(systems, Constants.armStow, ArmToGoalCommand.USE_INCHES | ArmToGoalCommand.FINISH_INSTANTLY));
        eventMap.put("stowLowCube", new ArmToGoalCommand(systems, Constants.armLowCube, ArmToGoalCommand.USE_INCHES | ArmToGoalCommand.FINISH_INSTANTLY));
        eventMap.put("stowLowCubeAfterBack", (ArmTrajectoryCommandFactory.procure(systems, Constants.ARM_TRAJECTORY_CONFIG_SLOW, Constants.armLowCube)));
        eventMap.put("armGroundCube", new ArmToGoalCommand(systems, Constants.armGroundCube, ArmToGoalCommand.USE_INCHES | ArmToGoalCommand.FINISH_INSTANTLY));
        eventMap.put("armBackwardsGroundCube", (ArmTrajectoryCommandFactory.procure(systems, Constants.ARM_TRAJECTORY_CONFIG_SLOW, Constants.armBackwardsGroundCube)));
        eventMap.put("armGroundUprightCone", new ArmToGoalCommand(systems, Constants.armGroundUprightCone, ArmToGoalCommand.USE_INCHES | ArmToGoalCommand.FINISH_INSTANTLY));
        eventMap.put("armGroundTippedCone", new ArmToGoalCommand(systems, Constants.armGroundTippedCone, ArmToGoalCommand.USE_INCHES | ArmToGoalCommand.FINISH_INSTANTLY));
    }

    public Command placeHighCone() {
        return sequence(
            new ArmToGoalCommand(
                systems,
                Constants.armWhileTraveling,
                ArmToGoalCommand.USE_INCHES
            ).withTimeout(0.5),
            new ArmToGoalCommand(
                systems,
                Constants.armHighCone,
                ArmToGoalCommand.USE_INCHES
            ).withTimeout(1),
            waitSeconds(0.1),
            new DriveCommand(systems, new ChassisSpeeds(-1.0, 0, 0)).withTimeout(0.75),
            systems.getManipulator().manipRunCommand(GamePiece.CUBE, false).withTimeout(0.5),
            new DriveCommand(systems, new ChassisSpeeds(1.0, 0, 0)).withTimeout(0.5),
            new ArmToGoalCommand(
                systems,
                Constants.armStow,
                ArmToGoalCommand.USE_INCHES)
        );
    }
    public Command placeHighCube() {
        return sequence(
            new ArmToGoalCommand(
                systems,
                Constants.armWhileTraveling,
                ArmToGoalCommand.USE_INCHES
            ).withTimeout(0.5),
            new ArmToGoalCommand(
                systems,
                Constants.armHighCube,
                ArmToGoalCommand.USE_INCHES
            ).withTimeout(1),
            waitSeconds(0.1),
            new DriveCommand(systems, new ChassisSpeeds(-1.0, 0, 0)).withTimeout(0.75),
            systems.getManipulator().manipRunCommand(GamePiece.CUBE, false).withTimeout(0.5),
            new DriveCommand(systems, new ChassisSpeeds(1.0, 0, 0)).withTimeout(0.5),
            new ArmToGoalCommand(
                systems,
                Constants.armStow,
                ArmToGoalCommand.USE_INCHES)
        );
    }
    public Command placeHighNoDrive(GamePiece holding) {
        return sequence(
            new ArmToGoalCommand(
                systems,
                Constants.armHighIntermediateOld,
                ArmToGoalCommand.USE_INCHES
            ).withTimeout(0.75),
            new ArmToGoalCommand(
                systems,
                Constants.armHighCone,
                ArmToGoalCommand.USE_INCHES
            ).withTimeout(1),
            waitSeconds(0.1),
            systems.getManipulator().manipRunCommand(holding, false).withTimeout(0.5)
        );
    }

    public Command placeMidCone() {
        return sequence(
            new ArmToGoalCommand(
                systems,
                Constants.armMidCone,
                ArmToGoalCommand.USE_INCHES
            ).withTimeout(0.75),
            waitSeconds(0.1),
            systems.getManipulator().manipRunCommand(GamePiece.CONE, false).withTimeout(0.5)
        );
    }

    public Command placeLowCube() {
        return sequence(
            new ArmToGoalCommand(
                systems,
                Constants.armLowCube,
                ArmToGoalCommand.USE_INCHES
            ).withTimeout(0.3),
            systems.getManipulator().manipRunCommand(GamePiece.CUBE, false).withTimeout(0.4)
        );
    }

    @Override
    public Command getAuto(String pathName) {
        if (pathName.equals("none")) return runOnce(() -> drivebase.resetGyroAt(180));
        if (pathName.equals("placeHigh")) return runOnce(() -> drivebase.resetGyroAt(180))
            .andThen(systems.getManipulator().manipRunOnceCommand(GamePiece.CUBE, true))
            .andThen(placeHighCube());
        if (pathName.equals("timedMobility")) return runOnce(() -> drivebase.resetGyroAt(180))
            .andThen(systems.getManipulator().manipRunOnceCommand(GamePiece.CUBE, true))
            .andThen(placeHighCube())
            .andThen(new DriveCommand(systems, new ChassisSpeeds(2.0, 0, 0)).withTimeout(3));
        if (pathName.equals("timedBalance")) return runOnce(() -> drivebase.resetGyroAt(180))
            .andThen(systems.getManipulator().manipRunOnceCommand(GamePiece.CUBE, true))
            .andThen(placeHighCube())
            .andThen(new DriveCommand(systems, new ChassisSpeeds(2.0, 0, 0)).withTimeout(1.2))
            .andThen(new AutobalancerHardcodePID(systems));

        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(pathName, Constants.PATH_CONSTRAINTS);

        Command setGyroCommand = new CommandBase() {
            Rotation2d rot2d = new Rotation2d();

            @Override
            public void initialize() {
                rot2d = PathPlannerTrajectory.transformStateForAlliance(pathGroup.get(0).getInitialState(), DriverStation.getAlliance()).holonomicRotation;
                drivebase.resetGyroAt(rot2d.getDegrees());
            }

            @Override
            public boolean isFinished() {
                // Wait until returned value matches set value
                return Math.abs(Rotation2d.fromDegrees(drivebase.pigeon2.getYaw()).minus(rot2d).getDegrees()) < 1;
            }

            @Override
            public Set<Subsystem> getRequirements() {
                return Set.of();
            }
        };
        // CommandBase setGyroCommand = none();

        // If running a path with two game pieces, start with a cone rather than a cube.
        Command setupIntakeCommand = (pathName.contains("TwoGP") || pathName.contains("ConeMid"))
            ? systems.getManipulator().manipRunOnceCommand(GamePiece.CONE, true)
            : systems.getManipulator().manipRunOnceCommand(GamePiece.CUBE, true);

        return setGyroCommand
        .andThen(setupIntakeCommand)
        .andThen(autoBuilder.fullAuto(pathGroup));
    }
}