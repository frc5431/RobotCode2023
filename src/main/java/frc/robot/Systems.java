package frc.robot;

import frc.robot.subsystems.*;

public class Systems {
    private Drivebase drivebase;

    private Vision vision;


    public Systems() {
        drivebase = new Drivebase();
        // vision = new Vision(drivebase);
    }

    public Drivebase getDrivebase() {
        return drivebase;
    }

    public Vision getVision() {
        return vision;
    }
}
