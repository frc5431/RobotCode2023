package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmToGoalCommand;
import frc.robot.commands.AutoAligner;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Vision;
import frc.robot.util.PresetPosition; 

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
    private final GenericEntry shouldBalance = Shuffleboard.getTab("Auton").add("Should Balance", false)
            .withWidget(BuiltInWidgets.kToggleButton).getEntry();

    public AutonLoader(Systems systems) {
        this.drivebase = systems.getDrivebase();

        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("deadwheelDrop", new RunCommand(() -> systems.getDeadwheels().deploy()));
        eventMap.put("deadwheelRaise", new RunCommand(() -> systems.getDeadwheels().retract()));
        eventMap.put("manipulatorOpen", new RunCommand(() -> systems.getManipulator().open()));
        eventMap.put("apriltagAlign", new RunCommand(() -> systems.getVision().detect()));
        eventMap.put("manipulatorGrab", new RunCommand(() -> systems.getManipulator().close()));
        eventMap.put("autoBalance", new RunCommand(() -> systems.getDeadwheels().retract()).andThen (new AutoAligner(drivebase)));        
            eventMap.put("placeHigh", new SequentialCommandGroup(systems.getArm().getWrist().setDegreesCommand(0)
            .andThen(new ArmToGoalCommand(systems,
                PresetPosition.fromGoal(new Translation2d(Constants.armHighX, Constants.armHighY), Constants.wristHighAngle),
                    ArmToGoalCommand.USE_INCHES | ArmToGoalCommand.FINISH_INSTANTLY)))
                        .andThen(new RunCommand(() -> systems.getManipulator().open()))
                            .andThen((PresetPosition.fromGoal(new Translation2d(Constants.armStowX, Constants.armStowY), Constants.wristStowAngle),
                                ArmToGoalCommand.USE_INCHES | ArmToGoalCommand.FINISH_INSTANTLY
                                    ))); 
                        
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

        Shuffleboard.getTab("Auton").add(chooser);
    }

    public Command getAuto(String pathName) {
        var pathGroup = PathPlanner.loadPathGroup(pathName, Constants.PATH_CONSTRAINTS);
        return autoBuilder.fullAuto(pathGroup);
    }

    public Command procureAuton() {
       var mostOfAuto = chooser.getSelected();

        if (shouldBalance.getBoolean(false)) {
            return mostOfAuto.andThen(new AutoAligner(drivebase));
        } else {
            return mostOfAuto;
        }
    }
}