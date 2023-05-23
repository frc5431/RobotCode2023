package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;

public class CircularLimit {
    double radius;

    public static double findLimit(double radius, double xy) {
        return Math.sqrt(Math.pow(radius, 2) - Math.pow(xy, 2));
    }

    public CircularLimit(double radius) {
        this.radius = radius;
    }

    public double findLimit(double y) {
        return CircularLimit.findLimit(this.radius, y);
    }

    private static double angleOfPoint(Translation2d point) {
        return Math.atan2(point.getY(), point.getX());
    }

    public static Translation2d pointOnCircle(double radius, double angle) {
        return new Translation2d(radius * Math.cos(angle), radius * Math.sin(angle));
    }

    public static Translation2d getClosestPointOnCircle(double radius, Translation2d point) {
        return pointOnCircle(radius, angleOfPoint(point));
    }

    public Translation2d getClosestPointOnCircle(Translation2d point) {
        return getClosestPointOnCircle(this.radius, point);
    }

    public boolean isPointInsideCircle(Translation2d point) {
        return Math.sqrt(Math.pow(point.getX(), 2) + Math.pow(point.getY(), 2)) <= radius;
    }
}
