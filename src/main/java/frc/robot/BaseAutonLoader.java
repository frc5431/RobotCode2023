package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivebase;

public abstract class BaseAutonLoader {

    public abstract String[] getPaths();

    public abstract void initializeEventMap(HashMap<String, Command> eventMap);


    protected Drivebase drivebase;

    /**
     * Gets initialized in {@link AutonLoader#AutonLoader(Drivebase)}.
     * 
     * IF YOU HAVE A NULLREFERENCE OR UNINITIALIZED BUG, IT'S POSSIBLY FROM HERE.
     */
    protected SwerveAutoBuilder autoBuilder;

    protected final SendableChooser<Command> chooser = new SendableChooser<>();
    private final HashMap<String, Command> eventMap = new HashMap<>();

    protected final Systems systems;

    public BaseAutonLoader(Systems systems) {
        this.drivebase = systems.getDrivebase();
        this.systems = systems;

        initializeEventMap(eventMap);
        
        autoBuilder = new SwerveAutoBuilder(
                drivebase::getEstimatedPosition,
                drivebase::resetOdometry,
                drivebase.m_kinematics,
                Constants.TRANSLATION_PID,
                Constants.ROTATION_PID,
                (states) -> drivebase.driveRaw(drivebase.m_kinematics.toChassisSpeeds(states)),
                eventMap,
                true,
                drivebase);

        for (String pathName : getPaths()) {
            chooser.addOption(pathName, getAuto(pathName));
        }

        SmartDashboard.putData("Auton", chooser);

        FieldObject2d targetPoseFieldObject = drivebase.field2d.getObject("TargetPose");
        FieldObject2d activeTrajectoryObject = drivebase.field2d.getObject("ActiveTrajectory");

        PPSwerveControllerCommand.setLoggingCallbacks(
        (activeTrajectory) -> {
            activeTrajectoryObject.setTrajectory(activeTrajectory);
        },
        (targetPose) -> {
            targetPoseFieldObject.setPose(targetPose);
        },
        null,
        (translationError, rotationError) -> {
            SmartDashboard.putNumber("PPSwerveControllerCommand/xErrorMeters", translationError.getX());
            SmartDashboard.putNumber("PPSwerveControllerCommand/yErrorMeters", translationError.getY());
            SmartDashboard.putNumber("PPSwerveControllerCommand/rotationErrorDegrees", rotationError.getDegrees());
        });
    }

    public abstract Command getAuto(String pathName);

    public Command procureAuton() {
       return chooser.getSelected();
    }
}