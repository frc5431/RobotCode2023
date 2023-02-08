package frc.robot.util;

import static edu.wpi.first.math.util.Units.radiansToDegrees;

public class PresetPosition {
    double outer = 0;
    double inner = 0;
    double wrist = 0;

    public double getOuter() {
        return outer;
    }
    public double getInner() {
        return inner;
    }
    public double getWrist() {
        return wrist;
    }

    // in degrees
    private PresetPosition(double outer, double inner, double wrist) {
        this.outer = outer;
        this.inner = inner;
        this.wrist = wrist;
    }

    public static PresetPosition fromRadians(double outer, double inner, double wrist){
        return new PresetPosition(radiansToDegrees(outer), radiansToDegrees(inner), radiansToDegrees(wrist));
    }

    public static PresetPosition fromDegrees(double outer, double inner, double wrist) {
        return new PresetPosition(outer, inner, wrist);
    }
}
