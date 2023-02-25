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

// TODO: Custom widget!!! That complains if you don't choose...

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
        eventMap.put("intakeDrop", new RunCommand(() ->
            systems.getDeadwheels().toggle()));
        eventMap.put("intakeRun", new RunCommand(() ->
            systems.getIntake().deploy()));
        eventMap.put("manipulatorOpen", new RunCommand(() -> systems.getManipulator().open()));
        eventMap.put("manipulatorGrab", new RunCommand(() -> systems.getManipulator().close()));
        eventMap.put("autoBalance", new RunCommand(() -> systems.getDeadwheels().retract()).andThen (new AutoAligner(drivebase)));
        eventMap.put("armGround", new ArmToGoalCommand(systems, new Translation2d(Constants.armGroundX, Constants.armGroundY),
            ArmToGoalCommand.FINISH_INSTANTLY | ArmToGoalCommand.USE_INCHES));
        eventMap.put("armInner", new ArmToGoalCommand(systems, new Translation2d(Constants.armInnerGrabX, Constants.armInnerGrabY), 
            ArmToGoalCommand.FINISH_INSTANTLY | ArmToGoalCommand.USE_INCHES));
        eventMap.put("armHigh", new ArmToGoalCommand(systems, new Translation2d(Constants.armHighX, Constants.armHighY),
            ArmToGoalCommand.FINISH_INSTANTLY | ArmToGoalCommand.USE_INCHES));
        eventMap.put("placeHigh", new SequentialCommandGroup(new ArmToGoalCommand(systems, new Translation2d(Constants.armHighX, Constants.armHighY),
            ArmToGoalCommand.FINISH_INSTANTLY | ArmToGoalCommand.USE_INCHES)
                .andThen(new RunCommand(() -> systems.getManipulator().open()))
                    .andThen(new ArmToGoalCommand(systems, new Translation2d(Constants.armInnerGrabX, Constants.armInnerGrabY),
                        ArmToGoalCommand.FINISH_INSTANTLY | ArmToGoalCommand.USE_INCHES)))); 
                        
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