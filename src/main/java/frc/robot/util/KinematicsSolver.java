package frc.robot.util;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;

public class KinematicsSolver {
    private double l1; // Length of first arm segment
    private double l2; // Length of second arm segment

    public KinematicsSolver(double armSegment1Length, double armSegment2Length) {
        this.l1 = armSegment1Length;
        this.l2 = armSegment2Length;
    }

    // Cos triangle rule: A^2 = B^2 + C^2 - 2BC * cos(a)
    // Isolate b: 

    private double solveSegment2Angle(Translation2d goal) {
        double p1 = Math.pow(goal.getX(),2) + Math.pow(goal.getY(),2) - Math.pow(l1,2) - Math.pow(l2,2);
        double p2 = 2*l1*l2;

        return Math.acos(p1/p2);
    }

    private double solveSegment1Angle(double q2, Translation2d goal) {
        double p1 = Math.atan2(goal.getY(), goal.getX());
        double p2 = Math.atan2(l2 * Math.sin(q2), l1 + l2 * Math.cos(q2));

        return p1 - p2;
    }

    public Pair<Double, Double> posToAngles(Translation2d goal) {
        double q2 = solveSegment2Angle(goal); //Replace once implemented
        double q1 = solveSegment1Angle(q2, goal);

        return Pair.of(q1 + Math.PI/2, -q2);
    }

    public Translation2d anglesToPos(double q1Radians, double q2Radians) {
        double fixed_q1 = q1Radians - Math.PI/2;
        double fixed_q2 = -q2Radians;
        double x = l1 * Math.cos(fixed_q1) + l2 * Math.cos(fixed_q1 + fixed_q2);
        double y = l1 * Math.sin(fixed_q1) + l2 * Math.sin(fixed_q1 + fixed_q2);
        return new Translation2d(x, y);
    }
}
