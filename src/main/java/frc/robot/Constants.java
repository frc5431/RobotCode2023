package frc.robot;

import java.io.IOException;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static final Transform3d CAMERA_OFFSET = new Transform3d(
        new Translation3d(Units.inchesToMeters(10.5), Units.inchesToMeters(11), Units.inchesToMeters(39.5)),
        new Rotation3d()
    );

    // public static final double CAMERA_HEIGHT_METERS = 0;

    public static final double LOW_APRILTAG_HEIGHT = 0.36;
    public static final double APRILTAG_HEIGHT = 0.59;
    public static final AprilTagFieldLayout layout = getLayout();

    public static final double armHighX = 40.875;
    public static final double armHighY = 27.66;
    public static final double wristHighAngle = 288;
    public static final double armMidX = 32.03;
    public static final double armMidY = 2.83;
    public static final double armInnerGrabX = 3.84;
    public static final double armInnerGrabY = -25.69;
    public static final double wristInvertAngle = 280;
    public static final double armGroundX = 6.17;
    public static final double armGroundY = -34.24;
    public static final double wristGroundAngle = 25;
    public static final double armStowX = 5.472;
    public static final double armStowY = -33;
    public static final double wristStowAngle = 105.32;
    public static final double armManipUnstuckX = -7;
    public static final double armManipUnstuckY = -30;


    static AprilTagFieldLayout getLayout() {
        try {
            return AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile);
        } catch (IOException e) {
            System.out.println("Apriltag Fields somehow did not intialize, if you are getting this error, you have bigger things to worry about though.");
            return null;
        }
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
    public static final PIDConstants ROTATION_PID = new PIDConstants(0.5, 0, 0);
    //#endregion Auto Constants

}
