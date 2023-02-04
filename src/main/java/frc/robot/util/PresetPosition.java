package frc.robot.util;

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

    public PresetPosition(double outer, double inner, double wrist) {
        this.outer = outer;
        this.inner = inner;
        this.wrist = wrist;
    }
}
