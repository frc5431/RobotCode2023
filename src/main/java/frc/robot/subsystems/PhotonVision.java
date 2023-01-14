package frc.robot.subsystems;

import frc.robot.Constants;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVision {
  public PhotonCamera camera = new PhotonCamera("photonvision");

  public void detectApriltagDistance() {
    var result = camera.getLatestResult();

    PhotonTrackedTarget targetA1 = result.getBestTarget();

    int id = targetA1.getFiducialId();


  }

}