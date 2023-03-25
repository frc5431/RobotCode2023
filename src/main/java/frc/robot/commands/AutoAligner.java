package frc.robot.commands;


import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Systems;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Vision;
import frc.team5431.titan.core.leds.Blinkin;

public class AutoAligner extends CommandBase {
    public final Drivebase drivebase;
    private final Blinkin leds;
    public final PhotonCamera camera;

    
    public AutoAligner(Drivebase drivebase, Blinkin leds, PhotonCamera camera)  {
        this.drivebase = drivebase;
        this.leds = leds;
        this.camera = camera;
    }
    
    @Override
    public void initialize() {
        leds.set(Constants.LEDAUTOALIGN);
    }

    @Override
    public void execute() {
        PhotonPipelineResult result = camera.getLatestResult();

        var imageCaptureTime = result.getTimestampSeconds();

        Double targPos =
        PhotonUtils.calculateDistanceToTargetMeters(
                Constants.CAMERA_HEIGHT_METERS,
                Constants.APRILTAG_HEIGHT,
                Constants.CAMERA_PITCH_RADIANS,
                Units.degreesToRadians(result.getBestTarget().getPitch()));

        if(result.hasTargets()){
            drivebase.addVisionMeasurement(targPos, imageCaptureTime);
            
        }
    }
    
}