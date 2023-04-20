package frc.robot.util;

import edu.wpi.first.math.Pair;

import static edu.wpi.first.math.util.Units.radiansToDegrees;
import static edu.wpi.first.math.util.Units.degreesToRadians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.ArmContainer;

/**
 * Default in inches, IF USED IN ArmToGoalCommand WITH USE_INCHES
 */
public class PresetPosition {
    private final double outer;
    private final double inner;
    private final double wrist;
    private final Translation2d wristPos;
    private final boolean useTopPossibility;

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
    public boolean getTopPossibility() {
        return useTopPossibility;
    }

    // in degrees
    private PresetPosition(double outer, double inner, double wrist, boolean useTopPossibility) {
        this.outer = outer;
        this.inner = inner;
        this.wristPos = ArmContainer.solver.anglesToPos(degreesToRadians(outer), degreesToRadians(inner));

        this.wrist = wrist;
        this.useTopPossibility = useTopPossibility;
    }
    
    // in degrees
    private PresetPosition(Translation2d wristPos, double wrist, boolean useTopPossibility) {
        this.wristPos = wristPos;
        
        Pair<Double, Double> result = ArmContainer.solver.posToAngles(wristPos, useTopPossibility);
        this.outer = result.getFirst();
        this.inner = result.getSecond();
        
        this.wrist = wrist;
        this.useTopPossibility = useTopPossibility;
    }
    
    public static PresetPosition fromRadians(double outer, double inner, double wrist, boolean useTopPossibility){
        return new PresetPosition(radiansToDegrees(outer), radiansToDegrees(inner), radiansToDegrees(wrist), useTopPossibility);
    }

    public static PresetPosition fromDegrees(double outer, double inner, double wrist, boolean useTopPossibility) {
        return new PresetPosition(outer, inner, wrist, useTopPossibility);
    }

    public static PresetPosition fromGoal(Translation2d wristPos, double wristDegrees, boolean useTopPossibility) {
        return new PresetPosition(wristPos, wristDegrees, useTopPossibility);
    }

    public static PresetPosition fromPose(Pose2d pose, boolean useTopPossibility) {
        return fromGoal(pose.getTranslation(), pose.getRotation().getDegrees(), useTopPossibility);
    }

    public Pose2d toPose2d() {
        return new Pose2d(wristPos, Rotation2d.fromDegrees(wrist));
    }

    public PresetPosition inchesToMeters() {
        return fromGoal(new Translation2d(Units.inchesToMeters(wristPos.getX()), Units.inchesToMeters(wristPos.getY())), wrist, useTopPossibility);
    }

    public PresetPosition metersToInches() {
        return fromGoal(new Translation2d(Units.metersToInches(wristPos.getX()), Units.metersToInches(wristPos.getY())), wrist, useTopPossibility);
    }

    public static boolean isGoalBackwards(Translation2d t) {
        return t.getX() < ArmContainer.IS_BACKWARDS_X;
    }

    public boolean isGoalBackwards() {
        return isGoalBackwards(wristPos);
    }

    @Override
    public String toString() {
        return String.format("(%f, %f) r: %f", wristPos.getX(), wristPos.getY(), wrist);
    }
}
