package frc.robot;

import java.io.File;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;

import org.opencv.highgui.HighGui;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.*;
import frc.robot.subsystems.Drivebase;

// init sendablechooser in robotcontainer's constructor ✅
// 	getfullauto retreives path groups
// 	(add them to the chooser)

// def getAutonomousCommand()
// {
// 	return chooser.getSelected()
// }

// TODO: map events to commands

// TODO: Custom widget!!! That complains if you don't choose...
public class AutonLoader {
    private Drivebase drivebase;

    /**
     * Gets initialized in {@link AutonLoader#AutonLoader(Drivebase)}.
     * 
     * IF YOU HAVE A NULLREFERENCE OR UNINITIALIZED BUG, IT'S POSSIBLY FROM HERE.
     */
    private SwerveAutoBuilder autoBuilder;

    private final SendableChooser<Command> chooser = new SendableChooser<>();
    private final GenericEntry shouldBalance = Shuffleboard.getTab("Auton").add("Balance", false)
            .withWidget(BuiltInWidgets.kBooleanBox).getEntry();

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
        JumpToGoalPositionCommand.FINISH_INSTANTLY | JumpToGoalPositionCommand.USE_INCHES)).andThen(new RunCommand(() -> Manipulator.open())
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
        
    
        ArrayList<String> pathsSim = new ArrayList<String>();
        pathsSim.add("far");
        pathsSim.add("farBalance");
        pathsSim.add("midBalance");
        pathsSim.add("near");
        pathsSim.add("nearBalance");

    /*         ArrayList<String> pathsRio = new ArrayList<String>();
    set     pathsRio.add("C:/Users/saddl/OneDrive/Documents/GitHub/RobotCode2023/src/main/deploy/pathplanner/far.path");
    this    pathsRio.add("C:/Users/saddl/OneDrive/Documents/GitHub/RobotCode2023/src/main/deploy/pathplanner/farBalance.path");
    to the  pathsRio.add("C:/Users/saddl/OneDrive/Documents/GitHub/RobotCode2023/src/main/deploy/pathplanner/midBalance.path");
    rio     pathsRio.add("C:/Users/saddl/OneDrive/Documents/GitHub/RobotCode2023/src/main/deploy/pathplanner/near.path");
    path    pathsRio.add("C:/Users/saddl/OneDrive/Documents/GitHub/RobotCode2023/src/main/deploy/pathplanner/nearBalance.path");
    */
s
        for (String pathNames : pathsSim) {
            chooser.addOption(pathNames, getFullAuto(pathNames));
        }

    }

    public Command getFullAuto(String pathName) {
        System.out.println(pathName);
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