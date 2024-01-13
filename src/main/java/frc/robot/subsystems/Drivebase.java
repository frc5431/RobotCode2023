package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.swervedrivespecialties.swervelib.MkModuleConfiguration;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

import java.util.ArrayList;
import java.util.List;

public class Drivebase extends SubsystemBase {
    
    /**
     * The maximum voltage that will be delivered to the drive motors.
     * <p>
     * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
     */
    public static final double MAX_VOLTAGE = 12.0;
    //  Measure the drivetrain's maximum velocity or calculate the theoretical.
    //  The formula for calculating the theoretical maximum velocity is:
    //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
    //  By default this value is setup for a Mk3 standard module using Falcon500s to drive.
    //  An example of this constant for a Mk4 L2 module with NEOs to drive is:
    //   5880.0 / 60.0 * SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
    /**
     * The maximum velocity of the robot in meters per second.
     * <p>
     * This is a measure of how fast the robot should be able to drive in a straight line.
     * 
     * 
     * MK4_L2 = 4.96823045476
     */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
                    SdsModuleConfigurations.MK4_L2.getDriveReduction() *
                    SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;
    /**
     * The maximum angular velocity of the robot in radians per second.
     * <p>
     * This is a measure of how fast the robot can rotate in place.
     */
    // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
                    Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

    public static final double MIN_ANGULAR_VELOCITY = 0.5;
    // Max input acceleration (ChassisSpeeds meters per second per second) for x/y movement
    public static final double SLEW_RATE_LIMIT_TRANSLATION = MAX_VELOCITY_METERS_PER_SECOND * 2;
    // Max input acceleration (ChassisSpeeds radians per second per second) for rotational movement
    public static final double SLEW_RATE_LIMIT_ROTATION = MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 10;

    public final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
                    // Front left
                    new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
                    // Front right
                    new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
                    // Back left
                    new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
                    // Back right
                    new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
    );
    
    public final Pigeon2 pigeon2;
    public final Pigeon2Configuration pigConfig = new Pigeon2Configuration();

    public final SwerveDrivePoseEstimator poseEstimator;

    // These are our modules. We initialize them in the constructor.
    private final SwerveModule m_frontLeftModule;
    private final SwerveModule m_frontRightModule;
    private final SwerveModule m_backLeftModule;
    private final SwerveModule m_backRightModule;

    private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    private final SlewRateLimiter filter_vx;
    private final SlewRateLimiter filter_vy;
    private final SlewRateLimiter filter_or;

    public final Field2d field2d;

    public Drivebase() {
        pigeon2 = new Pigeon2(ID_PIGEON2, CANBUS_DRIVETRAIN);
        // m_pigeon2.configFactoryDefault();
        MountPoseConfigs config = new MountPoseConfigs();
        config.MountPosePitch = 0;
        config.MountPoseRoll = 0;
        config.MountPoseYaw = 180;
        pigeon2.getConfigurator().apply(config);
        //pigeon2.configMountPose(AxisDirection.NegativeX, AxisDirection.PositiveZ);
        // m_pigeon2.zeroGyroBiasNow(200);

        MkModuleConfiguration moduleConfig = MkModuleConfiguration.getDefaultSteerFalcon500();
        moduleConfig.setDriveCurrentLimit(40.0);
        moduleConfig.setSteerCurrentLimit(30.0);

        m_frontLeftModule = new MkSwerveModuleBuilder(moduleConfig)
                // .withLayout(getSMLayout(tab.getLayout("Front Left Module", BuiltInLayouts.kList))
                //         .withPosition(0, 0))
                .withGearRatio(SdsModuleConfigurations.MK4_L2)
                .withDriveMotor(MotorType.FALCON, FRONT_LEFT_MODULE_DRIVE_MOTOR, CANBUS_DRIVETRAIN)
                .withSteerMotor(MotorType.FALCON, FRONT_LEFT_MODULE_STEER_MOTOR, CANBUS_DRIVETRAIN)
                .withSteerEncoderPort(FRONT_LEFT_MODULE_STEER_ENCODER, CANBUS_DRIVETRAIN)
                .withSteerOffset(FRONT_LEFT_MODULE_STEER_OFFSET)
                .build();

        // We will do the same for the other modules
        m_frontRightModule = new MkSwerveModuleBuilder(moduleConfig)
                // .withLayout(getSMLayout(tab.getLayout("Front Right Module", BuiltInLayouts.kList))
                //         .withPosition(3, 0))
                .withGearRatio(SdsModuleConfigurations.MK4_L2)
                .withDriveMotor(MotorType.FALCON, FRONT_RIGHT_MODULE_DRIVE_MOTOR, CANBUS_DRIVETRAIN)
                .withSteerMotor(MotorType.FALCON, FRONT_RIGHT_MODULE_STEER_MOTOR, CANBUS_DRIVETRAIN)
                .withSteerEncoderPort(FRONT_RIGHT_MODULE_STEER_ENCODER, CANBUS_DRIVETRAIN)
                .withSteerOffset(FRONT_RIGHT_MODULE_STEER_OFFSET)
                .build();

        m_backLeftModule = new MkSwerveModuleBuilder(moduleConfig)
                // .withLayout(getSMLayout(tab.getLayout("Back Left Module", BuiltInLayouts.kList))
                //         .withPosition(6, 0))
                .withGearRatio(SdsModuleConfigurations.MK4_L2)
                .withDriveMotor(MotorType.FALCON, BACK_LEFT_MODULE_DRIVE_MOTOR, CANBUS_DRIVETRAIN)
                .withSteerMotor(MotorType.FALCON, BACK_LEFT_MODULE_STEER_MOTOR, CANBUS_DRIVETRAIN)
                .withSteerEncoderPort(BACK_LEFT_MODULE_STEER_ENCODER, CANBUS_DRIVETRAIN)
                .withSteerOffset(BACK_LEFT_MODULE_STEER_OFFSET)
                .build();

        m_backRightModule = new MkSwerveModuleBuilder(moduleConfig)
                // .withLayout(getSMLayout(tab.getLayout("Back Right Module", BuiltInLayouts.kList))
                //         .withPosition(9, 0))
                .withGearRatio(SdsModuleConfigurations.MK4_L2)
                .withDriveMotor(MotorType.FALCON, BACK_RIGHT_MODULE_DRIVE_MOTOR, CANBUS_DRIVETRAIN)
                .withSteerMotor(MotorType.FALCON, BACK_RIGHT_MODULE_STEER_MOTOR, CANBUS_DRIVETRAIN)
                .withSteerEncoderPort(BACK_RIGHT_MODULE_STEER_ENCODER, CANBUS_DRIVETRAIN)
                .withSteerOffset(BACK_RIGHT_MODULE_STEER_OFFSET)
                .build();

        poseEstimator = new SwerveDrivePoseEstimator(m_kinematics, getGyroscopeRotation(), getPositions(), new Pose2d());
        

        filter_vx = new SlewRateLimiter(SLEW_RATE_LIMIT_TRANSLATION);
        filter_vy = new SlewRateLimiter(SLEW_RATE_LIMIT_TRANSLATION);
        filter_or = new SlewRateLimiter(SLEW_RATE_LIMIT_ROTATION);

        // ShuffleboardLayout chassisSpeedsLayout = tab_subsystems.getLayout("ChassisSpeeds", BuiltInLayouts.kList)
        //         .withSize(2, 3)
        //         .withPosition(21, 0);
        // chassisSpeedsLayout.addNumber("vX", () -> m_chassisSpeeds.vxMetersPerSecond);
        // chassisSpeedsLayout.addNumber("vY", () -> m_chassisSpeeds.vyMetersPerSecond);
        // chassisSpeedsLayout.addNumber("oR", () -> m_chassisSpeeds.omegaRadiansPerSecond);

        SmartDashboard.putData("Gyro", pigeon2);

        ShuffleboardTab visionTab = Shuffleboard.getTab("Vision");

        field2d = new Field2d();

        visionTab.addString("Pose", this::getFormattedPose)
            .withPosition(0, 0)
            .withSize(2, 0);
        visionTab.add("Field", field2d)
            .withPosition(2, 0)
            .withSize(6,4);
    }

    private String getFormattedPose() {
        Pose2d pose = getEstimatedPosition();
        return String.format("(%.2f, %.2f) %.2f degrees",
            pose.getX(),
            pose.getY(),
            pose.getRotation().getDegrees());
    }


    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(getGyroscopeRotation(), getPositions(), pose);
    }

    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
    }

    public Pose2d getEstimatedPosition() {
        return poseEstimator.getEstimatedPosition();
    }


    public void zeroGyroscope() {
        pigeon2.reset();
        resetOdometry(getEstimatedPosition());
    }

    public void resetGyroAt(double yaw) {
        pigeon2.setYaw(yaw);
    }

    public Rotation2d getGyroscopeRotation() {
        return pigeon2.getRotation2d();
    }

    public Pigeon2 getGyro() {
        return pigeon2;
    }


    public void drive(ChassisSpeeds chassisSpeeds) {
        ChassisSpeeds speedsModified = new ChassisSpeeds(
            filter_vx.calculate(chassisSpeeds.vxMetersPerSecond),
            filter_vy.calculate(chassisSpeeds.vyMetersPerSecond),
            filter_or.calculate(chassisSpeeds.omegaRadiansPerSecond)
        );
        driveRaw(speedsModified);
    }

    public void driveRaw(ChassisSpeeds chassisSpeeds) {
        m_chassisSpeeds = chassisSpeeds;
    }
    public void stop() {
        driveRaw(new ChassisSpeeds());
    }

    public SwerveModulePosition[] getPositions() {
        return new SwerveModulePosition[] {
            m_frontLeftModule.getPosition(),
            m_frontRightModule.getPosition(),
            m_backLeftModule.getPosition(),
            m_backRightModule.getPosition()
        };
    }

    public List<TalonFX> getMotors() {
        List<TalonFX> retval = new ArrayList<>();
        for (SwerveModule s : new SwerveModule[]{ m_frontLeftModule, m_frontRightModule, m_backLeftModule, m_backRightModule }) {
            retval.add((TalonFX) s.getSteerMotor());
            retval.add((TalonFX) s.getDriveMotor());
        }
        return retval;
    }


    @Override
    public void periodic() {
        poseEstimator.update(getGyroscopeRotation(), getPositions());
        field2d.setRobotPose(getEstimatedPosition());
        SmartDashboard.putNumber("Pitch", pigeon2.getPitch().getValueAsDouble());
        
        final double zeroDeadzone = 0.001;

        // Set deadzone on translation
        if (Math.abs(m_chassisSpeeds.vxMetersPerSecond) < zeroDeadzone) {
            m_chassisSpeeds.vxMetersPerSecond = 0;
        }
        if (Math.abs(m_chassisSpeeds.vyMetersPerSecond) < zeroDeadzone) {
            m_chassisSpeeds.vyMetersPerSecond = 0;
        }

        // Hockey-lock if stopped by setting rotation to realllly low number
        if (m_chassisSpeeds.vxMetersPerSecond == 0 && 
            m_chassisSpeeds.vyMetersPerSecond == 0 && 
            Math.abs(m_chassisSpeeds.omegaRadiansPerSecond) < zeroDeadzone) {
            m_chassisSpeeds.omegaRadiansPerSecond = 0.00001;
        }

        SmartDashboard.putNumber("DT X spd", m_chassisSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("DT Y spd", m_chassisSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber("DT . spd", Math.hypot(m_chassisSpeeds.vxMetersPerSecond, m_chassisSpeeds.vyMetersPerSecond));
        SmartDashboard.putNumber("DT O rot", m_chassisSpeeds.omegaRadiansPerSecond);

        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

        SwerveModulePosition[] positions = getPositions();
        for (int i = 0; i < states.length; i++) {
            states[i] = SwerveModuleState.optimize(states[i], positions[i].angle);
        }

        double flVoltage;
        double frVoltage;
        double blVoltage;
        double brVoltage;

        flVoltage = states[0].speedMetersPerSecond;
        frVoltage = states[1].speedMetersPerSecond;
        blVoltage = states[2].speedMetersPerSecond;
        brVoltage = states[3].speedMetersPerSecond;

        // flVoltage = MathUtil.clamp(flVoltage, 0, MAX_VELOCITY_METERS_PER_SECOND);
        // frVoltage = MathUtil.clamp(frVoltage, 0, MAX_VELOCITY_METERS_PER_SECOND);
        // blVoltage = MathUtil.clamp(blVoltage, 0, MAX_VELOCITY_METERS_PER_SECOND);
        // brVoltage = MathUtil.clamp(brVoltage, 0, MAX_VELOCITY_METERS_PER_SECOND);

        // SmartDashboard.putNumber("Front Left Velocity", flVoltage);
        // SmartDashboard.putNumber("Front Right Velocity", frVoltage);
        // SmartDashboard.putNumber("Back Left Velocity", blVoltage);
        // SmartDashboard.putNumber("Back Right Velocity", brVoltage);

        flVoltage = flVoltage / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE;
        frVoltage = frVoltage / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE;
        blVoltage = blVoltage / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE;
        brVoltage = brVoltage / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE;

        m_frontLeftModule.set(flVoltage, states[0].angle.getRadians());
        m_frontRightModule.set(frVoltage, states[1].angle.getRadians());
        m_backLeftModule.set(blVoltage, states[2].angle.getRadians());
        m_backRightModule.set(brVoltage, states[3].angle.getRadians());
    }
}
