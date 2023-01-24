package frc.robot.subsystems;

import edu.wpi.first.math.Pair;
import frc.robot.Constants;

import java.util.Optional;

import frc.robot.Robot;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  public Drivebase drivebase;
  public PhotonCamera camera = new PhotonCamera("photonvision");

  public Vision(Drivebase drivebase) {
    this.drivebase = drivebase;
    Robot.periodics.add(new Pair<>(this::detect, 0.2)); // 5 times per sec
  }
  

//  public PhotonTrackedTarget getResult() {
//    var result = camera.getLatestResult();
//    return result.getBestTarget();
//  }

  public Pose3d getAprilTagPosition(PhotonTrackedTarget targetA1) {
    int id = targetA1.getFiducialId();
    assert Constants.layout != null;
    Optional<Pose3d> apriltagPosition = Constants.layout.getTagPose(id);

    return apriltagPosition.orElse(null);
  }

//  public void sendResultDrivebase(Pose3d absPos, Pose3d relPos, double timestamp) {
//    drivebase.addVisionMeasurement(relPos.relativeTo(absPos).toPose2d(), timestamp);
//  }

//  public void detect() {
//    PhotonTrackedTarget result = getResult();
//    if(result == null) {
//      return;
//    }
//
//    Pose3d relPos = Conversions.tPose3d(result.getBestCameraToTarget()).transformBy(Constants.CAMERA_OFFSET);
//    Pose3d absPos = getAprilTagPosition(result);
//    sendResultDrivebase(absPos, relPos, camera.getLatestResult().getTimestampSeconds());
//  }

  public void detect() {
    var res = camera.getLatestResult();

    if (res.hasTargets()) {

      var imageCaptureTime = res.getTimestampSeconds();

      var camToTargetTrans = res.getBestTarget().getBestCameraToTarget();

      var camPose = getAprilTagPosition(res.getBestTarget()).transformBy(camToTargetTrans.inverse());

      drivebase.addVisionMeasurement(
            camPose.transformBy(Constants.CAMERA_OFFSET).toPose2d(), imageCaptureTime
      );

    }
  }
}