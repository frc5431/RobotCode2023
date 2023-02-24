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
import frc.robot.commands.AutoAligner;
import frc.robot.commands.JumpToGoalPositionCommand;
import frc.robot.subsystems.Drivebase;

// init sendablechooser in robotcontainer's constructor ✅
// 	getfullauto retreives path groups
// 	(add them to the chooser)
// TODO: map events to commands
// TODO: Custom widget!!! That complains if you don't choose...

//public ArrayList<String> balancePaths = new ArrayList<String>();
//balancePaths.add("farBalance");
//balancePaths.add("midBalance");
//balancePaths.add("nearBalance");



public class AutonLoader {

    private static final String[] paths = {
    "far", "near"
    };

    private static final String[] balancePaths = {
    "farBalance", "middleBalance", "nearBalance"
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
        eventMap.put("deadwheelDrop", new RunCommand(() -> systems.getDeadwheels().toggle()));
        eventMap.put("deadwheelRaise", new RunCommand(() -> systems.getDeadwheels().toggle()));
        // eventMap.put("intakeDrop", new RunCommand(() ->
        // systems.getDeadwheels().toggle()));
        // eventMap.put("intakeRun", new RunCommand(() ->
        // systems.getIntake().deploy()));
        eventMap.put("manipulatorOpen", new RunCommand(() -> systems.getManipulator().open()));
        eventMap.put("manipulatorGrab", new RunCommand(() -> systems.getManipulator().close()));
        eventMap.put("autoBalance", new AutoAligner(drivebase));
        eventMap.put("armGround", new JumpToGoalPositionCommand(systems.getArm(), new Translation2d(6.17, -34.24),
                JumpToGoalPositionCommand.FINISH_INSTANTLY | JumpToGoalPositionCommand.USE_INCHES));
        eventMap.put("armInner", new JumpToGoalPositionCommand(systems.getArm(), new Translation2d(3.84, -25.69),
                JumpToGoalPositionCommand.FINISH_INSTANTLY | JumpToGoalPositionCommand.USE_INCHES));
        eventMap.put("armHigh", new JumpToGoalPositionCommand(systems.getArm(), new Translation2d(40.875, 27.66),
                JumpToGoalPositionCommand.FINISH_INSTANTLY | JumpToGoalPositionCommand.USE_INCHES));
        eventMap.put("placeHigh", new SequentialCommandGroup( new JumpToGoalPositionCommand(systems.getArm(), new Translation2d(40.875, 27.66),
        JumpToGoalPositionCommand.FINISH_INSTANTLY | JumpToGoalPositionCommand.USE_INCHES)).andThen(new RunCommand(() -> systems.getManipulator().open())
        )); 

        // This can be reused for all autos.
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
                chooser.addOption(pathNames, getFullAuto(pathNames));
        }

        Shuffleboard.getTab("Auton").add(chooser);
    }

    public Command getFullAuto(String pathName) {
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