package frc.robot;

import frc.robot.subsystems.*;

public class Systems {
    private Drivebase drivebase;

    public Systems() {
        drivebase = new Drivebase();
    }

    public Drivebase getDrivebase() {
        return drivebase;
    }
}
