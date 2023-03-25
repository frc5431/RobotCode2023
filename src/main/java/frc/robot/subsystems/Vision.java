package frc.robot.subsystems;

import frc.robot.Constants;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  public Drivebase drivebase;
  public PhotonCamera camera = new PhotonCamera("OV5647");

  public Vision(Drivebase drivebase) {
    this.drivebase = drivebase;
    // Robot.periodics.add(new Pair<>(this::detect, 0.2)); // 5 times per sec
  }

  public Pose3d getAprilTagPosition(PhotonTrackedTarget targetA1) {
    Optional<Pose3d> apriltagPosition = Constants.layout.getTagPose(targetA1.getFiducialId());

    return apriltagPosition.orElse(null);
  }

  public void travel() {
    var res = camera.getLatestResult();

    if (res.hasTargets()) {

      var imageCaptureTime = res.getTimestampSeconds();

      var camToTargetTrans = res.getBestTarget().getBestCameraToTarget();

      Pose3d atpos = getAprilTagPosition(res.getBestTarget());
      if (atpos == null) return;

      var camPose = atpos.transformBy(camToTargetTrans.inverse());

      drivebase.addVisionMeasurement(
            camPose.transformBy(Constants.CAMERA_OFFSET).toPose2d(), imageCaptureTime
      );

    }
  }

  

}