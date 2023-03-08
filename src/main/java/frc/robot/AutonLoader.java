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
import frc.robot.commands.ArmMoveCommandGroup;
import frc.robot.commands.ArmToGoalCommand;
import frc.robot.commands.Autobalancer;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Drivebase;
import frc.robot.util.PresetPosition;

import static edu.wpi.first.wpilibj2.command.Commands.*;

public class AutonLoader {

    private static final String[] paths = {
        "far", "farBalance",
        "mid", "midBalance",
        "near", "nearBalance",
        "special",
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

    public AutonLoader(Systems systems) {
        this.drivebase = systems.getDrivebase();

        HashMap<String, Command> eventMap = new HashMap<>();
        // eventMap.put("deadwheelDrop", systems.getDeadwheels().deadwheelsCommand(true));
        // eventMap.put("deadwheelRaise", systems.getDeadwheels().deadwheelsCommand(false));
        eventMap.put("deadwheelDrop", none());
        eventMap.put("deadwheelRaise", none());
        eventMap.put("manipulatorOpen", systems.getManipulator().manipCommand(true));
        eventMap.put("manipulatorGrab", systems.getManipulator().manipCommand(false));
        eventMap.put("autoBalance", new Autobalancer(systems));        
        eventMap.put("placeHigh", new SequentialCommandGroup(
            systems.getArm().getWrist().setDegreesCommand(0),
            new ArmMoveCommandGroup( // Arm while traveling
                systems,
                new Translation2d(14.34, -11.95),
                ArmToGoalCommand.USE_INCHES,
                302,
                false
            ).withTimeout(1),
            new ArmMoveCommandGroup( // midpoint
                systems,
                new Translation2d(25, -2),
                ArmToGoalCommand.USE_INCHES,
                295,
                false
            ).withTimeout(1),
            new ArmToGoalCommand(
                systems,
                PresetPosition.fromGoal(new Translation2d(Constants.armHighX, Constants.armHighY), Constants.wristHighAngle),
                ArmToGoalCommand.USE_INCHES).withTimeout(2),
            waitSeconds(0.2),
            new DriveCommand(systems, new ChassisSpeeds(-0.75, 0, 0)).withTimeout(1),
            systems.getManipulator().manipCommand(true),
            waitSeconds(1.5),
            new DriveCommand(systems, new ChassisSpeeds(0.75, 0, 0)).withTimeout(1.05),
            new ArmToGoalCommand(
                systems,
                PresetPosition.fromGoal(new Translation2d(Constants.armStowX, Constants.armStowY), Constants.wristStowAngle),
                ArmToGoalCommand.USE_INCHES)
        ));
        eventMap.put("stow", new ArmToGoalCommand(
            systems,
            PresetPosition.fromGoal(new Translation2d(Constants.armStowX, Constants.armStowY), Constants.wristStowAngle),
            ArmToGoalCommand.USE_INCHES | ArmToGoalCommand.FINISH_INSTANTLY
        ));
        // eventMap.put("placeHigh", none());

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

    public Command getAuto(String pathName) {
        if (pathName.equals("none")) return runOnce(() -> drivebase.resetGyroAt(180));

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