package frc.robot;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmGoalGroup;
import frc.robot.commands.ArmToGoalCommand;
import frc.robot.commands.Autobalancer;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Manipulator.GamePiece;
import frc.robot.util.PresetPosition;

import static edu.wpi.first.wpilibj2.command.Commands.*;

public class AutonLoader {

    private static final String[] paths = {
        "far", "farBalance",
        "mid", "midBalance",
        "near", "nearBalance",
        "special", "placeHigh",
        "timedMobility",
        "timedBalance",
        "none"
    };


    private Drivebase drivebase;

    /**
     * Gets initialized in {@link AutonLoader#AutonLoader(Drivebase)}.
     * 
     * IF YOU HAVE A NULLREFERENCE OR UNINITIALIZED BUG, IT'S POSSIBLY FROM HERE.
     */
    private SwerveAutoBuilder autoBuilder;

    private final SendableChooser<Command> chooser = new SendableChooser<>();
    private final HashMap<String, Command> eventMap = new HashMap<>();

    private final Systems systems;

    public AutonLoader(Systems systems) {
        this.drivebase = systems.getDrivebase();
        this.systems = systems;

        eventMap.put("deadwheelDrop", none());
        eventMap.put("deadwheelRaise", none());
        eventMap.put("manipulatorOpen", systems.getManipulator().manipRunOnceCommand(GamePiece.CUBE, false));
        eventMap.put("manipulatorGrab", systems.getManipulator().manipRunOnceCommand(GamePiece.CUBE, true));
        eventMap.put("autoBalance", new Autobalancer(systems));
        eventMap.put("placeHigh", placeHigh());
        // eventMap.put("placeHigh", none());
        eventMap.put("stow", new ArmToGoalCommand(
            systems,
            Constants.armStow,
            ArmToGoalCommand.USE_INCHES | ArmToGoalCommand.FINISH_INSTANTLY
        ));

        autoBuilder = new SwerveAutoBuilder(
                drivebase::getPosition,
                drivebase::resetOdometry,
                drivebase.m_kinematics,
                Constants.TRANSLATION_PID,
                Constants.ROTATION_PID,
                (states) -> drivebase.driveRaw(drivebase.m_kinematics.toChassisSpeeds(states)),
                eventMap,
                true,
                drivebase);

        for (String pathNames : paths) {
                chooser.addOption(pathNames, getAuto(pathNames));
        }

        SmartDashboard.putData("Auton", chooser);
    }
//s
    public Command placeHigh() {
        return new SequentialCommandGroup(
            systems.getArm().getWrist().setDegreesCommand(0), // while traveling
            new ArmGoalGroup(
                systems,
                PresetPosition.fromGoal(new Translation2d(8.54, -5.73), 308, false),
                ArmToGoalCommand.USE_INCHES | ArmToGoalCommand.FINISH_INSTANTLY
            ).withTimeout(1),
            // new ArmMoveCommandGroup( // midpoint
            //     systems,
            //     new Translation2d(25, -2),
            //     ArmToGoalCommand.USE_INCHES,
            //     295,
            //     false
            // ).withTimeout(1),
            new ArmToGoalCommand(
                systems,
                Constants.armHigh,
                ArmToGoalCommand.USE_INCHES).withTimeout(1.4),
            waitSeconds(0.1),
            new DriveCommand(systems, new ChassisSpeeds(-1.0, 0, 0)).withTimeout(0.6),
            systems.getManipulator().manipRunCommand(GamePiece.CUBE, false).withTimeout(0.5),
            new DriveCommand(systems, new ChassisSpeeds(1.0, 0, 0)).withTimeout(0.5),
            new ArmToGoalCommand(
                systems,
                Constants.armStow,
                ArmToGoalCommand.USE_INCHES)
        );
    }

    public Command getAuto(String pathName) {
        if (pathName.equals("none")) return runOnce(() -> drivebase.resetGyroAt(180));
        if (pathName.equals("placeHigh")) return runOnce(() -> drivebase.resetGyroAt(180)).andThen(placeHigh());
        if (pathName.equals("timedMobility")) return runOnce(() -> drivebase.resetGyroAt(180))
            .andThen(placeHigh())
            .andThen(new DriveCommand(systems, new ChassisSpeeds(2.0, 0, 0)).withTimeout(3));
        if (pathName.equals("timedBalance")) return runOnce(() -> drivebase.resetGyroAt(180))
            .andThen(placeHigh())
            .andThen(new DriveCommand(systems, new ChassisSpeeds(2.0, 0, 0)).withTimeout(0.5))
            .andThen(new Autobalancer(systems));

        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(pathName, Constants.PATH_CONSTRAINTS);

        CommandBase setGyroCommand = runOnce(() -> {
            Rotation2d rot2d = PathPlannerTrajectory.transformStateForAlliance(pathGroup.get(0).getInitialState(), DriverStation.getAlliance()).holonomicRotation;
            drivebase.resetGyroAt(rot2d.getDegrees());
        });

        return setGyroCommand.andThen(autoBuilder.fullAuto(pathGroup));
    }

    public Command procureAuton() {
       return chooser.getSelected();
    }
}