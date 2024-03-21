package frc.robot;

// import com.pathplanner.lib.PathConstraints;
// import com.pathplanner.lib.auto.PIDConstants;
import java.util.List;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

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
            new Rotation3d());
    public static final Transform3d CAMERA_TO_ROBOT = ROBOT_TO_CAMERA.inverse();

    // public static final double CAMERA_HEIGHT_METERS = 0;

    public static final double LOW_APRILTAG_HEIGHT = 0.36;
    public static final double APRILTAG_HEIGHT = 0.59;

    public static final TrajectoryConfig ARM_TRAJECTORY_CONFIG_SLOW = new TrajectoryConfig(2, 2);
    public static final TrajectoryConfig ARM_TRAJECTORY_CONFIG = new TrajectoryConfig(6, 10);

    // Needs to be converted from inches to meters as it is not passed through
    // ArmToGoalCommand.USE_INCHES
    public static final PresetPosition armBackwardsGroundCube = PresetPosition
            .fromGoal(new Translation2d(-29.09, -33.005), 277, false);
    public static final PresetPosition armHighIntermediate = PresetPosition.fromGoal(new Translation2d(28.21, 6.09), 0,
            false);
    public static final PresetPosition armToBackIntermediate = PresetPosition
            .fromGoal(new Translation2d(-30.39, -16.15), 0, false);

    public static final PresetPosition armStow = PresetPosition.fromGoal(new Translation2d(0.5705, -25.55), 323.12,
            false);
    public static final PresetPosition armHighCone = PresetPosition.fromGoal(new Translation2d(40.1, 9), 312.2, false);
    public static final PresetPosition armHighCube = PresetPosition.fromGoal(armHighCone.getWristPos(), 230, false);
    public static final PresetPosition armMidCone = PresetPosition.fromGoal(new Translation2d(23.08, -1.19), 284,
            false);
    public static final PresetPosition armHighIntermediateOld = PresetPosition.fromGoal(new Translation2d(18.59, 1.14),
            260.6, false);
    public static final PresetPosition armMidCube = PresetPosition.fromGoal(new Translation2d(19.881, -1.94), 228.3,
            false);
    public static final PresetPosition armLowCube = PresetPosition.fromGoal(Constants.armStow.getWristPos(), 244,
            false);
    public static final PresetPosition armGroundTippedCone = PresetPosition.fromGoal(new Translation2d(13.05, -38.25),
            33, false);
    public static final PresetPosition armGroundCube = PresetPosition.fromGoal(new Translation2d(9.07, -30.05), 260,
            false);
    public static final PresetPosition armGroundCubeWithin = PresetPosition.fromGoal(new Translation2d(-.4, -33.78),
            312.94, false);
    public static final PresetPosition armGroundUprightCone = PresetPosition
            .fromGoal(new Translation2d(3.7691, -22.909), 290, false);
    public static final PresetPosition armSingleSubPickup = PresetPosition.fromGoal(new Translation2d(20.694, -7.11),
            345.2, false);
    public static final PresetPosition armDoubleSubPickup = PresetPosition.fromGoal(new Translation2d(26.426, -0.59),
            297.47, false);
    public static final PresetPosition armDoubleSubStandingPickup = PresetPosition
            .fromGoal(new Translation2d(34.96, 16.70), 224.44, false);
    public static final PresetPosition armWhileTraveling = PresetPosition.fromGoal(new Translation2d(8.54, -5.73), 259,
            false);

    public static class OldTunerConstatns {
        // Both sets of gains need to be tuned to your individual robot.

        // The steer motor uses any SwerveModule.SteerRequestType control request with
        // the
        // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
        private static final Slot0Configs steerGains = new Slot0Configs()
                .withKP(100).withKI(0).withKD(0.3)
                .withKS(0).withKV(1.5).withKA(0);
        // When using closed-loop control, the drive motor uses the control
        // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
        private static final Slot0Configs driveGains = new Slot0Configs()
                .withKP(1.6).withKI(0).withKD(0)
                .withKS(0).withKV(0).withKA(0);

        // The closed-loop output type to use for the steer motors;
        // This affects the PID/FF gains for the steer motors
        private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
        // The closed-loop output type to use for the drive motors;
        // This affects the PID/FF gains for the drive motors
        private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

        // The stator current at which the wheels start to slip;
        // This needs to be tuned to your individual robot
        private static final double kSlipCurrentA = 300.0;

        // Theoretical free speed (m/s) at 12v applied output;
        // This needs to be tuned to your individual robot
        public static final double kSpeedAt12VoltsMps = (6000 / 60.0) *
                SdsModuleConfigurations.MK4_L2.getDriveReduction() *
                SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;

        public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.5842;

        public static final double DRIVETRAIN_WHEELBASE_METERS = 0.5334;

        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = kSpeedAt12VoltsMps / Math.hypot(
                DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

        // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
        // This may need to be tuned to your individual robot
        private static final double kCoupleRatio = 3.5714285714285714285714285714286;

        private static final double kDriveGearRatio = 6.746031746031747;
        private static final double kSteerGearRatio = 12.8;
        private static final double kWheelRadiusInches = 2;

        private static final boolean kSteerMotorReversed = false;
        private static final boolean kInvertLeftSide = false;
        private static final boolean kInvertRightSide = false;

        private static final String kCANbusName = CANBUS_DRIVETRAIN;
        private static final int kPigeonId = 13;

        // These are only used for simulation
        private static final double kSteerInertia = 0.00001;
        private static final double kDriveInertia = 0.001;
        // Simulated voltage necessary to overcome friction
        private static final double kSteerFrictionVoltage = 0.25;
        private static final double kDriveFrictionVoltage = 0.25;

        public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
                .withPigeon2Id(kPigeonId)
                .withCANbusName(kCANbusName);

        private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
                .withDriveMotorGearRatio(kDriveGearRatio)
                .withSteerMotorGearRatio(kSteerGearRatio)
                .withWheelRadius(kWheelRadiusInches)
                .withSlipCurrent(kSlipCurrentA)
                .withSteerMotorGains(steerGains)
                .withDriveMotorGains(driveGains)
                .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
                .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
                .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
                .withSteerInertia(kSteerInertia)
                .withDriveInertia(kDriveInertia)
                .withSteerFrictionVoltage(kSteerFrictionVoltage)
                .withDriveFrictionVoltage(kDriveFrictionVoltage)
                .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
                .withCouplingGearRatio(kCoupleRatio)
                .withSteerMotorInverted(kSteerMotorReversed);
        /**
         * How to re-align swerves & find offsets manually.
         * 1. set all 'k[...]EncoderOffset' to '-(0)'
         * 2. deploy
         * 3. power cycle
         * 4. drive forward, and then adjust all swerve modules to point forward
         * 5. in pheonix tuner x select the CANCoder and get the "Absolute Position No
         * Offset"
         * 6. set the 'k[...]EncoderOffset' to '-(value)'
         * 7. deploy
         * 8. power cycle
         * 9. profit!
         * 
         */

        // Front Left
        private static final int kFrontLeftDriveMotorId = 4;
        private static final int kFrontLeftSteerMotorId = 3;
        private static final int kFrontLeftEncoderId = 9;
        private static final double kFrontLeftEncoderOffset = -(-0.232422);

        private static final double kFrontLeftXPosInches = 10.25;
        private static final double kFrontLeftYPosInches = 11.75;

        // Front Right
        private static final int kFrontRightDriveMotorId = 8;
        private static final int kFrontRightSteerMotorId = 7;
        private static final int kFrontRightEncoderId = 12;
        private static final double kFrontRightEncoderOffset = -(-0.025391);

        private static final double kFrontRightXPosInches = 10.25;
        private static final double kFrontRightYPosInches = -11.75;

        // Back Left
        private static final int kBackLeftDriveMotorId = 6;
        private static final int kBackLeftSteerMotorId = 5;
        private static final int kBackLeftEncoderId = 11;
        private static final double kBackLeftEncoderOffset = -(-0.154785);

        private static final double kBackLeftXPosInches = -10.25;
        private static final double kBackLeftYPosInches = 11.75;

        // Back Right
        private static final int kBackRightDriveMotorId = 2;
        private static final int kBackRightSteerMotorId = 1;
        private static final int kBackRightEncoderId = 9;
        private static final double kBackRightEncoderOffset = -(-0.189941);

        private static final double kBackRightXPosInches = -10.25;
        private static final double kBackRightYPosInches = -11.75;

        public static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
                kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset,
                Units.inchesToMeters(kFrontLeftXPosInches), Units.inchesToMeters(kFrontLeftYPosInches),
                kInvertLeftSide);
        public static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
                kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset,
                Units.inchesToMeters(kFrontRightXPosInches), Units.inchesToMeters(kFrontRightYPosInches),
                kInvertRightSide);
        public static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
                kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset,
                Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches), kInvertLeftSide);
        public static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
                kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset,
                Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches),
                kInvertRightSide);

    }

    // public static final double armBackwardsHighX = -58.35; // -59.7
    // public static final double armBackwardsHighY = 12.1; // 3.7
    // public static final double wristBackwardsHighAngle = 12; // 340
    // public static final PresetPosition armBackwardsHigh =
    // PresetPosition.fromGoal(new Translation2d(armBackwardsHighX,
    // armBackwardsHighY), wristBackwardsHighAngle, false);
    // public static final double armBackwardsMidX = -42.58;
    // public static final double armBackwardsMidY = -11;
    // public static final double wristBackwardsMidAngle = 335.7;
    // public static final PresetPosition armBackwardsMid =
    // PresetPosition.fromGoal(new Translation2d(armBackwardsMidX,
    // armBackwardsMidY), wristBackwardsMidAngle, false);
    public static class ApriltagConstants {
        // Team
        public static List<Integer> blue = List.of(6, 7, 8, 9, 10, 14, 15, 16);
        public static List<Integer> red = List.of(1, 2, 3, 4, 5, 11, 12, 13);

        // Retreival
        public static List<Integer> source = List.of(9, 10, 1, 2);

        // Scoring
        public static List<Integer> amp = List.of(5, 6);
        public static List<Integer> speaker = List.of(7, 8, 3, 4);

        // well this one's obvious
        public static List<Integer> stage = List.of(11, 12, 13, 14, 15, 16);
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
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(0.249268 * 360); // 293.906

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 8;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 7;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 12;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0.284668 * 360); // 0.284668 rotations

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 4;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 3;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 10;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(0.147461 * 360); // 111.533

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 6;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 5;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 11;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0.860352 * 360); // 53.701

    // #region Auto Constants
    // Pretty sure constraints from the path file are not used, and the following
    // is.
    // public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(4,
    // 3);
    // public static final PIDConstants TRANSLATION_PID = new PIDConstants(6, 0,
    // 0.04);
    // public static final PIDConstants ROTATION_PID = new PIDConstants(2.5, 0, 0);
    // //#endregion Auto Constants

}
