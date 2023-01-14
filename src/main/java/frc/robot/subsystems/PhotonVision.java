package frc.robot.subsystems;
import edu.wpi.first.math.geometry.Transform2d;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import java.util.List;

public class PhotonVision {
  public PhotonCamera awesomeCamera1 = new PhotonCamera("photonvision");

  // gets latest from A1, then creates target for A1
  public void getLatestA1(){
    var resultA1 = awesomeCamera1.getLatestResult();
    //may be helpfull
    System.out.println(resultA1);
    //honastaly I don't know the point of this
    PhotonTrackedTarget targetA1 = resultA1.getBestTarget();
    //right value
    double yaw = targetA1.getYaw();
    //height value
    double pitch = targetA1.getPitch();
    //area value
    double area = targetA1.getArea();
    //the ~s~k~e~w~
    double skew = targetA1.getSkew();
//    Transform2d pose = targetA1.replace();
  //  List<TargetCorner> corners = targetA1.getCorners();} }
    }


}