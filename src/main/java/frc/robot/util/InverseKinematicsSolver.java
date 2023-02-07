package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class InverseKinematicsSolver {
    private double l1; // Length of first arm segment
    private double l2; // Length of second arm segment

    public InverseKinematicsSolver(double armSegment1Length, double armSegment2Length) {
        this.l1 = armSegment1Length;
        this.l2 = armSegment2Length;
    }

    // Cos triangle rule: A^2 = B^2 + C^2 - 2BC * cos(a)
    // Isolate b: 

    double solveSegment1Angle(double q2, Translation2d goal) {
        double p1 = Math.atan2(goal.getY(), goal.getX());
        double p2 = Math.atan2(l2 * Math.sin(q2), l1 + l2 * Math.cos(q2));

        return p1 - p2;
    }

    PresetPosition solveForPosition(Translation2d goal) {
        double outerAngle = 0.; //Replace once implemented
        double innerAngle = solveSegment1Angle(outerAngle, goal);

        return new PresetPosition(outerAngle, innerAngle, 0);
    }
}
