package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Conversions;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVision extends SubsystemBase {
  public Drivebase drivebase;
  public PhotonCamera camera = new PhotonCamera("photonvision");

  public PhotonVision(Drivebase drivebase) {
    this.drivebase = drivebase;
  }
  

  public PhotonTrackedTarget getResult(){
    var result = camera.getLatestResult();
    PhotonTrackedTarget targetA1 = result.getBestTarget();
    return targetA1;
  }

  public Pose3d getAprilTagPosition(PhotonTrackedTarget targetA1){
    int id = targetA1.getFiducialId();

    Optional<Pose3d> ApriltagPosition = Constants.layout.getTagPose(id);

    if(ApriltagPosition.isEmpty()) {
      return null;
    }
    return ApriltagPosition.get();
  }
  
  public void sendResultDrivebase(Pose3d AbsPos, Pose3d RelPos) {
    drivebase.addVisionMeasurement(RelPos.relativeTo(AbsPos).toPose2d(), Timer.getFPGATimestamp());
  }

  @Override

  public void periodic() {
      PhotonTrackedTarget result = getResult();
      Pose3d RelPos = Conversions.tPose3d(result.getBestCameraToTarget());
      Pose3d AbsPos = getAprilTagPosition(result);
      sendResultDrivebase(AbsPos, RelPos);
  }
}