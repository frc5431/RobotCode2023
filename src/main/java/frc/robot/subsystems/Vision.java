package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Robot;

import java.io.IOException;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  public Drivebase drivebase;
  public PhotonCamera camera = new PhotonCamera("OV5647");
  public final AprilTagFieldLayout LAYOUT;

  private double previousTimestamp = 0;

  public Vision(Drivebase drivebase) {
    this.drivebase = drivebase;
    AprilTagFieldLayout layout;
    try {
      layout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
      layout.setOrigin(switch (DriverStation.getAlliance()) {
        case Blue -> OriginPosition.kBlueAllianceWallRightSide;
        default -> OriginPosition.kRedAllianceWallRightSide;
      });
    } catch (IOException e) {
      DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
      layout = null;
    }
    this.LAYOUT = layout;
    Robot.periodics.add(new Pair<>(this::detect, 0.2)); // 5 times per sec
  }

  public void detect() {
    PhotonPipelineResult res = camera.getLatestResult();
    double timestamp = res.getTimestampSeconds();

    if (timestamp != previousTimestamp && res.hasTargets()) {
      previousTimestamp = timestamp;
      PhotonTrackedTarget target = res.getBestTarget();
      int id = target.getFiducialId();

      Optional<Pose3d> tagPose = LAYOUT == null ? Optional.empty() : LAYOUT.getTagPose(id);

      if (target.getPoseAmbiguity() <= .2 && id >= 0 && tagPose.isPresent()) {
        Pose3d targetPose = tagPose.get();
        Transform3d camToTarget = target.getBestCameraToTarget();
        Pose3d camPose = targetPose.transformBy(camToTarget.inverse());

        Pose3d visionMeasurement = camPose.transformBy(Constants.CAMERA_TO_ROBOT);
        drivebase.addVisionMeasurement(visionMeasurement.toPose2d(), timestamp);
      }
    }
  }
}