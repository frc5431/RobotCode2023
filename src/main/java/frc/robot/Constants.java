package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
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

    public static final PresetPosition armHigh = PresetPosition.fromGoal(new Translation2d(36.53, 10.37), 305, false);
    public static final PresetPosition armMid = PresetPosition.fromGoal(new Translation2d(18.59, 1.14), 260.6, false);
    public static final PresetPosition armNormalGrabOld = PresetPosition.fromGoal(new Translation2d(6.17, -34.24), 15, false);
    public static final PresetPosition armGroundCube = PresetPosition.fromGoal(new Translation2d(9.07, -30.05), 271.15, false);
    public static final PresetPosition armGroundUprightCone = PresetPosition.fromGoal(new Translation2d(3.58, -22.59), 290, false);
    public static final PresetPosition armSingleSubPickup = PresetPosition.fromGoal(new Translation2d(20.694, -7.11), 345.2, false);
    public static final PresetPosition armWhileTraveling = PresetPosition.fromGoal(new Translation2d(8.54, -5.73), 308, false);
    public static final PresetPosition armStow = PresetPosition.fromGoal(new Translation2d(2.49, -19.85), 305, false);
    // public static final double armBackwardsHighX = -58.35; // -59.7
    // public static final double armBackwardsHighY = 12.1; // 3.7
    // public static final double wristBackwardsHighAngle = 12; // 340
    // public static final PresetPosition armBackwardsHigh = PresetPosition.fromGoal(new Translation2d(armBackwardsHighX, armBackwardsHighY), wristBackwardsHighAngle, false);
    // public static final double armBackwardsMidX = -42.58;
    // public static final double armBackwardsMidY = -11;
    // public static final double wristBackwardsMidAngle = 335.7;
    // public static final PresetPosition armBackwardsMid = PresetPosition.fromGoal(new Translation2d(armBackwardsMidX, armBackwardsMidY), wristBackwardsMidAngle, false);
    // public static final PresetPosition armBackwardsIntermediate = PresetPosition.fromGoal(new Translation2d(-13.89, -1.7), 260.0, false);

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

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 2;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 1;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 9;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(293.906); // 264.375

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 8;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 7;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 12;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(108.281); // 106.699

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 4;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 3;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 10;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(111.533); // 330.645

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 6;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 5;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 11;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(53.701); // 115.664

    //#region Auto Constants
    public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(4, 3);
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(5, 0, 0);
    public static final PIDConstants ROTATION_PID = new PIDConstants(1.0, 0, 0);
    //#endregion Auto Constants

}
