package frc.robot.util;

import edu.wpi.first.math.Pair;

import static edu.wpi.first.math.util.Units.radiansToDegrees;
import static edu.wpi.first.math.util.Units.degreesToRadians;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.Arm;
public class PresetPosition {
    private final double outer;
    private final double inner;
    private final double wrist;
    private final Translation2d wristPos;

    public double getOuter() {
        return outer;
    }
    public double getInner() {
        return inner;
    }
    public double getWrist() {
        return wrist;
    }

    public Translation2d getWristPos() {
        return wristPos;
    }

    // in degrees
    private PresetPosition(double outer, double inner, double wrist) {
        this.outer = outer;
        this.inner = inner;
        this.wristPos = Arm.solver.anglesToPos(degreesToRadians(outer), degreesToRadians(inner));

        this.wrist = wrist;
    }

    // in degrees
    private PresetPosition(Translation2d wristPos, double wrist) {
        this.wristPos = wristPos;

        Pair<Double, Double> result = Arm.solver.posToAngles(wristPos);
        this.outer = result.getFirst();
        this.inner = result.getSecond();

        this.wrist = wrist;
    }
    
    public static PresetPosition fromRadians(double outer, double inner, double wrist){
        return new PresetPosition(radiansToDegrees(outer), radiansToDegrees(inner), radiansToDegrees(wrist));
    }

    public static PresetPosition fromDegrees(double outer, double inner, double wrist) {
        return new PresetPosition(outer, inner, wrist);
    }

    public static PresetPosition fromGoal(Translation2d wristPos, double wristDegrees) {
        return new PresetPosition(wristPos, wristDegrees);
    }

    public static boolean isGoalBackwards(Translation2d t) {
        return t.getX() < Arm.IS_BACKWARDS_X;
    }

    public boolean isGoalBackwards() {
        return isGoalBackwards(wristPos);
    }
}
