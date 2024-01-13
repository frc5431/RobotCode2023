package frc.robot;

// import com.pathplanner.lib.PathConstraints;
// import com.pathplanner.lib.auto.PIDConstants;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;
import frc.robot.util.PresetPosition;

public class Constants {
    // Location of camera on the robot, relative to the center of the robot.
    // (X forward, Y left, Z up)
    public static final Transform3d ROBOT_TO_CAMERA = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(10.5),
            Units.inchesToMeters(-11),
            Units.inchesToMeters(39.5)),
        new Rotation3d()
    );
    public static final Transform3d CAMERA_TO_ROBOT = ROBOT_TO_CAMERA.inverse();

    // public static final double CAMERA_HEIGHT_METERS = 0;

    public static final double LOW_APRILTAG_HEIGHT = 0.36;
    public static final double APRILTAG_HEIGHT = 0.59;

    public static final TrajectoryConfig ARM_TRAJECTORY_CONFIG_SLOW = new TrajectoryConfig(2, 2);
    public static final TrajectoryConfig ARM_TRAJECTORY_CONFIG = new TrajectoryConfig(6, 10);

    // Needs to be converted from inches to meters as it is not passed through ArmToGoalCommand.USE_INCHES
    public static final PresetPosition armBackwardsGroundCube = PresetPosition.fromGoal(new Translation2d(-29.09, -33.005), 277, false);
    public static final PresetPosition armHighIntermediate = PresetPosition.fromGoal(new Translation2d(28.21, 6.09), 0, false);
    public static final PresetPosition armToBackIntermediate = PresetPosition.fromGoal(new Translation2d(-30.39, -16.15), 0, false);

    public static final PresetPosition armStow = PresetPosition.fromGoal(new Translation2d(0.5705,-25.55), 323.12, false);
    public static final PresetPosition armHighCone = PresetPosition.fromGoal(new Translation2d(40.1, 9), 312.2, false);
    public static final PresetPosition armHighCube = PresetPosition.fromGoal(armHighCone.getWristPos(), 230, false);
    public static final PresetPosition armMidCone = PresetPosition.fromGoal(new Translation2d(23.08, -1.19), 284, false);
    public static final PresetPosition armHighIntermediateOld = PresetPosition.fromGoal(new Translation2d(18.59, 1.14), 260.6, false);
    public static final PresetPosition armMidCube = PresetPosition.fromGoal(new Translation2d(19.881, -1.94), 228.3, false);
    public static final PresetPosition armLowCube = PresetPosition.fromGoal(Constants.armStow.getWristPos(), 244, false);
    public static final PresetPosition armGroundTippedCone = PresetPosition.fromGoal(new Translation2d(13.05, -38.25), 33, false);
    public static final PresetPosition armGroundCube = PresetPosition.fromGoal(new Translation2d(9.07, -30.05), 260, false);
    public static final PresetPosition armGroundCubeWithin = PresetPosition.fromGoal(new Translation2d(-.4, -33.78), 312.94, false);
    public static final PresetPosition armGroundUprightCone = PresetPosition.fromGoal(new Translation2d(3.7691, -22.909), 290, false);
    public static final PresetPosition armSingleSubPickup = PresetPosition.fromGoal(new Translation2d(20.694, -7.11), 345.2, false);
    public static final PresetPosition armDoubleSubPickup = PresetPosition.fromGoal(new Translation2d(26.426, -0.59), 297.47, false);
    public static final PresetPosition armDoubleSubStandingPickup = PresetPosition.fromGoal(new Translation2d(34.96, 16.70), 224.44, false);
    public static final PresetPosition armWhileTraveling = PresetPosition.fromGoal(new Translation2d(8.54, -5.73), 259, false);
    
    // public static final double armBackwardsHighX = -58.35; // -59.7
    // public static final double armBackwardsHighY = 12.1; // 3.7
    // public static final double wristBackwardsHighAngle = 12; // 340
    // public static final PresetPosition armBackwardsHigh = PresetPosition.fromGoal(new Translation2d(armBackwardsHighX, armBackwardsHighY), wristBackwardsHighAngle, false);
    // public static final double armBackwardsMidX = -42.58;
    // public static final double armBackwardsMidY = -11;
    // public static final double wristBackwardsMidAngle = 335.7;
    // public static final PresetPosition armBackwardsMid = PresetPosition.fromGoal(new Translation2d(armBackwardsMidX, armBackwardsMidY), wristBackwardsMidAngle, false);
    public static class ApriltagConstants {
        // Team
        public static List<Integer> blue = List.of(6,7,8,9,10,14,15,16);
        public static List<Integer> red = List.of(1,2,3,4,5,11,12,13);
    
        // Retreival
        public static List<Integer> source = List.of(9,10,1,2);
    
        // Scoring
        public static List<Integer> amp = List.of(5,6);
        public static List<Integer> speaker = List.of(7,8,3,4);
    
        // well this one's obvious
        public static List<Integer> stage = List.of(11,12,13,14,15,16);
    }
    public static final int ID_PIGEON2 = 13;
    public static final int ID_PHUB = 1;

    public static final String CANBUS_DRIVETRAIN = "omnivore"; // "omnivore"
    public static final String CANBUS_SUBSYSTEM = "";

    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.546;
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.648;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 4; // 2
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 3; // 1
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 10; // 9
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(89.736+180); //291.888

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 8;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 7;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 12;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(103.184); //461.602
                                                                                            

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 6; // 4 
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 5; // 3
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 11; // 10
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(66.006 + 180); // 110.654
                                                                                         //267.363
    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 2; // 6
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 1; // 5
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 9; // 11
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(112.412); // -201.445

    //#region Auto Constants
    // Pretty sure constraints from the path file are not used, and the following is.
    // public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(4, 3);
    // public static final PIDConstants TRANSLATION_PID = new PIDConstants(6, 0, 0.04);
    // public static final PIDConstants ROTATION_PID = new PIDConstants(2.5, 0, 0);
    // //#endregion Auto Constants

}
