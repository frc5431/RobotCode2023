package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ArmToGoalCommand;
import frc.robot.commands.Autobalancer;
import frc.robot.subsystems.Drivebase;
import frc.robot.util.PresetPosition;

import static edu.wpi.first.wpilibj2.command.Commands.*;

public class AutonLoader {

    private static final String[] paths = {
    "far", "farBalance", "mid", "midBalance", "near", "nearBalance"
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
        eventMap.put("deadwheelDrop", systems.getDeadwheels().deadwheelsCommand(true));
        eventMap.put("deadwheelRaise", systems.getDeadwheels().deadwheelsCommand(false));
        eventMap.put("manipulatorOpen", systems.getManipulator().manipCommand(true));
        eventMap.put("manipulatorGrab", systems.getManipulator().manipCommand(false));
        eventMap.put("autoBalance", new Autobalancer(systems));        
        // eventMap.put("placeHigh", new SequentialCommandGroup(
        //     systems.getArm().getWrist().setDegreesCommand(0),
        //     new ArmToGoalCommand(
        //         systems,
        //         PresetPosition.fromGoal(new Translation2d(Constants.armHighX, Constants.armHighY), Constants.wristHighAngle),
        //         ArmToGoalCommand.USE_INCHES),
        //     systems.getManipulator().manipCommand(true),
        //     new ArmToGoalCommand(
        //         systems,
        //         PresetPosition.fromGoal(new Translation2d(Constants.armStowX, Constants.armStowY), Constants.wristStowAngle),
        //         ArmToGoalCommand.USE_INCHES | ArmToGoalCommand.FINISH_INSTANTLY)
        // ));
        eventMap.put("stow", new ArmToGoalCommand(
            systems,
            PresetPosition.fromGoal(new Translation2d(Constants.armStowX, Constants.armStowY), Constants.wristStowAngle),
            ArmToGoalCommand.USE_INCHES | ArmToGoalCommand.FINISH_INSTANTLY
        ));
        eventMap.put("placeHigh", none());

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
        var pathGroup = PathPlanner.loadPathGroup(pathName, Constants.PATH_CONSTRAINTS);
        return autoBuilder.fullAuto(pathGroup);
    }

    public Command procureAuton() {
       return chooser.getSelected();
    }
}