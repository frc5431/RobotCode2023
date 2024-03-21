package frc.robot.subsystems;

import java.util.ConcurrentModificationException;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Drivebase extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private SwerveRequest.ApplyChassisSpeeds drive = new SwerveRequest.ApplyChassisSpeeds();

    public Drivebase(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(8.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(12.0, 0.0, 0.0), // Rotation PID constants
                        2.5, // Max module speed m/s
                        0.5125830761935194, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig(true, false) // Default path replanning config. See the API for the options here
                ),
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
        
    }

    public Pose2d getPose(){
        return this.getState().Pose;
    }

    public void resetPose(Pose2d newPose){
        this.seedFieldRelative(newPose);
    }
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return this.m_kinematics.toChassisSpeeds(this.getState().ModuleStates);       
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        this.setControl(drive.withSpeeds(speeds));
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Pigeon2 getGyro() {
        return this.m_pigeon2;
    }

    public void set(double angle) {
        getPigeon2().setYaw(angle);
        resetPose(getPose());
    }

    public void zeroGyro() {
        getPigeon2().reset();
        resetPose(getPose());
    }

    Field2d undertale = new Field2d();

    @Override
    public void periodic() {
        
        // // SmartDashboard.putNumber("X Speed", this.getRobotRelativeSpeeds().vxMetersPerSecond);
        // // SmartDashboard.putNumber("Y Speed", this.getRobotRelativeSpeeds().vyMetersPerSecond);

        // m_odometry.update(m_fieldRelativeOffset, m_modulePositions);
        // // Get estimated poses from VisionSubsystem
        // var visionEstimatedRobotPoses = LasaVision.getInstance().getEstimatedGlobalPoses();

        // // // Exit if no valid vision pose estimates
        // if (visionEstimatedRobotPoses.isEmpty()) return;

        // try {
        //     // Add vision measurements to pose estimator
        //     for (var visionEstimatedRobotPose : visionEstimatedRobotPoses) {
        //         // if (visionEstimatedRobotPose.estimatedPose.toPose2d().getTranslation().getDistance(m_previousPose.getTranslation()) > 1.0) continue;
        //         m_odometry.addVisionMeasurement(visionEstimatedRobotPose.estimatedPose.toPose2d(), visionEstimatedRobotPose.timestampSeconds);
        //     }
        // }catch(ConcurrentModificationException e) {
        //     // i am going to throw this dumb robot out a window
        // }

        // undertale.setRobotPose(m_odometry.getEstimatedPosition());
        // // SmartDashboard.putData("Field", undertale);
    }

    public SwerveDrivePoseEstimator getOdometry() {
        return m_odometry;
    }    


    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at 
        a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
}
