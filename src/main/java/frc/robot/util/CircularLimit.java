package frc.robot.util;

public class CircularLimit {
    double radius;

    public static double findLimit(double radius, double y) {
        return Math.sqrt(Math.pow(radius, 2) - Math.pow(y, 2));
    }

    public CircularLimit(double radius) {
        this.radius = radius;
    }

    public double findLimit(double y) {
        return CircularLimit.findLimit(this.radius, y);
    }
}
