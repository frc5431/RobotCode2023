package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Robot;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    public Drivebase drivebase;
    public PhotonCamera camera;
    public PhotonPoseEstimator photonPoseEstimator;
    public final AprilTagFieldLayout LAYOUT;

    public Vision(Drivebase drivebase) {
        this.drivebase = drivebase;
        camera = new PhotonCamera("OV5647");
        AprilTagFieldLayout layout;
        try {
            layout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
            layout.setOrigin(switch (DriverStation.getAlliance()) {
                case Blue -> OriginPosition.kBlueAllianceWallRightSide;
                default -> OriginPosition.kRedAllianceWallRightSide;
            });

            photonPoseEstimator = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP, camera, Constants.ROBOT_TO_CAMERA);
            photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        } catch (IOException e) {
            DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
            layout = null;
            photonPoseEstimator = null;
        }
        this.LAYOUT = layout;
        Robot.periodics.add(new Pair<>(this::detect, 0.2)); // 5 times per sec
    }

    /**
     * @param estimatedRobotPose The current best guess at robot pose
     * @return an EstimatedRobotPose with an estimated pose, the timestamp, and targets used to create
     *     the estimate
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        if (photonPoseEstimator == null) {
            // The field layout failed to load, so we cannot estimate poses.
            return Optional.empty();
        }
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }

    public void detect() {
        Optional<EstimatedRobotPose> result = getEstimatedGlobalPose(drivebase.getEstimatedPosition());

        if (result.isPresent()) {
            EstimatedRobotPose camPose = result.get();
            drivebase.addVisionMeasurement(
                    camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
            drivebase.field2d.getObject("Cam Est Pos").setPose(camPose.estimatedPose.toPose2d());
        } else {
            // move it way off the screen to make it disappear
            drivebase.field2d.getObject("Cam Est Pos").setPose(new Pose2d(-100, -100, new Rotation2d()));
        }


        // PhotonPipelineResult res = camera.getLatestResult();
        // double timestamp = res.getTimestampSeconds();

        // if (timestamp != previousTimestamp && res.hasTargets()) {
        //     previousTimestamp = timestamp;
        //     PhotonTrackedTarget target = res.getBestTarget();
        //     int id = target.getFiducialId();

        //     Optional<Pose3d> tagPose = LAYOUT == null ? Optional.empty() : LAYOUT.getTagPose(id);

        //     if (target.getPoseAmbiguity() <= .2 && id >= 0 && tagPose.isPresent()) {
        //         Pose3d targetPose = tagPose.get();
        //         Transform3d camToTarget = target.getBestCameraToTarget();
        //         Pose3d camPose = targetPose.transformBy(camToTarget.inverse());

        //         Pose3d visionMeasurement = camPose.transformBy(Constants.CAMERA_TO_ROBOT);
        //         drivebase.addVisionMeasurement(visionMeasurement.toPose2d(), timestamp);
        //     }
        // }
    }
}