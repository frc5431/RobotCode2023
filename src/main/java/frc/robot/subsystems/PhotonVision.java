package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Conversions;
import frc.robot.Robot;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVision extends SubsystemBase {
  public Drivebase drivebase;
  public PhotonCamera camera = new PhotonCamera("photonvision");

  public PhotonVision(Drivebase drivebase) {
    this.drivebase = drivebase;
  }
  

  public void updateVisionMeasurement() {
    var result = camera.getLatestResult();

    PhotonTrackedTarget targetA1 = result.getBestTarget();
    if(targetA1 == null) {
      return;
    }

    int id = targetA1.getFiducialId();

    Optional<Pose3d> ApriltagPosition = Constants.layout.getTagPose(id);

    if(ApriltagPosition.isEmpty()) {
      return;
    }

    Pose3d AbsPos = ApriltagPosition.get();
    Pose3d RelPos = Conversions.tPose3d(targetA1.getBestCameraToTarget());

    drivebase.addVisionMeasurement(RelPos.relativeTo(AbsPos).toPose2d(), Timer.getFPGATimestamp());
  }

  @Override
  public void periodic() {
      // TODO Auto-generated method stub
      super.periodic();
      
      updateVisionMeasurement();
  }

}