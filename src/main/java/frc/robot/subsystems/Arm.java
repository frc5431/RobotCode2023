package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.RunEndCommand;

import static edu.wpi.first.math.geometry.Rotation2d.fromDegrees;
import static edu.wpi.first.math.geometry.Rotation2d.fromRadians;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.math.util.Units.radiansToDegrees;

public class Arm extends SubsystemBase {
    
    private final CANSparkMax outerArm;
    private final CANSparkMax outerArmFollow;
    private final CANSparkMax innerArm;
    private final CANSparkMax innerArmFollow;

    private final SparkMaxPIDController outerController;
    private final AbsoluteEncoder outerEncoder;

    private final SparkMaxPIDController innerController;
    private final AbsoluteEncoder innerEncoder;

    private final CANSparkMax wrist;
    private final PIDController wristController;
    private final AbsoluteEncoder wristEncoder;

    public static final double MAX_SPEED_OUTER = 0.8;
    public static final double MAX_SPEED_INNER = 0.3;
    public static final double MAX_SPEED_WRIST = 0.5;

    private double setpointOut = 0;
    private double setpointIn = 0;
    private double setpointWrist = 0;

    private static final Rotation2d DEG_90 = fromDegrees(90);
    private static final double TORQUE_NM_NEO = 2.6;

    public static final double SHOULDER_TORQUE_TOTAL = TORQUE_NM_NEO * 60 * 2;
    public static final double FOREARM_TORQUE_TOTAL = TORQUE_NM_NEO * 5 * 4.5 * 2;
    public static final double WRIST_TORQUE_TOTAL = TORQUE_NM_NEO * 9;

    // measured
    private Rotation2d bicepAngle = new Rotation2d();
    private Rotation2d bicepAngleToGround = new Rotation2d();
    private Rotation2d forearmAngle = new Rotation2d();
    private Rotation2d forearmAngleToGround = new Rotation2d();
    private Rotation2d handAngle = new Rotation2d();
    private Rotation2d handAngleToGround = new Rotation2d();

    // ((mass (kg) * acceleration (m/s/s)) (N) * distance of center of mass from pivot (m)) (Nm)
    public static final double shoulderMinCosineMultiplier =
        12 * 9.81 * Units.inchesToMeters(22);

    public static final double shoulderMaxCosineMultiplier =
        12 * 9.81 * Units.inchesToMeters(50);

    public static final double forearmMinCosineMultiplier =
        3.8 * 9.81 * Units.inchesToMeters(22);

    public static final double forearmMaxCosineMultiplier =
        3.8 * 9.81 * Units.inchesToMeters(22.93);

    public static final double wristCosineMultiplier = 
        1.95 * 9.81 * Units.inchesToMeters(3.1);

    /* Arm encoder directions (robot facing right)
     * - shoulder
     *   - encoder CCW+
     *   - 0 facing down
     * - elbow
     *   - motor CW+
     *   - encoder CW+
     *   - 0 facing away from bicep
     * - wrist
     *   - motor CCW+
     *   - encoder CCW+
     *   - 0 facing away from forearm
     */

    public Arm(CANSparkMax outerArmLeft, CANSparkMax outerArmRight, CANSparkMax innerArmLeft, CANSparkMax innerArmRight, CANSparkMax wrist) {
        this.outerArm = outerArmLeft;
        this.outerArmFollow = outerArmRight;
        this.innerArm = innerArmLeft;
        this.innerArmFollow = innerArmRight;
        this.wrist = wrist;

        outerArm.setInverted(false);
        outerArmFollow.follow(outerArm, true);
        outerArm.setIdleMode(IdleMode.kBrake);
        outerArmFollow.setIdleMode(IdleMode.kBrake);

        outerController = this.outerArm.getPIDController();
        outerEncoder = this.outerArm.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

        outerController.setP(0.5);
        // controller.setI(0.0002);
        outerController.setI(0.0);
        outerController.setD(0.0);
        outerController.setFF(0.00);
        outerController.setOutputRange(-MAX_SPEED_OUTER, MAX_SPEED_OUTER);

        outerController.setFeedbackDevice(outerEncoder);

        innerArm.setInverted(true);
        innerArmFollow.follow(innerArm, true);
        innerArm.setIdleMode(IdleMode.kBrake);
        innerArmFollow.setIdleMode(IdleMode.kBrake);

        innerArm.enableSoftLimit(SoftLimitDirection.kForward, false);
        innerArm.enableSoftLimit(SoftLimitDirection.kReverse, false);

        // innerArm.setSoftLimit(SoftLimitDirection.kForward, 0.80f);
        // innerArm.setSoftLimit(SoftLimitDirection.kReverse, 0.20f);

        innerController = innerArm.getPIDController();
        innerEncoder = innerArm.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

        innerController.setP(1.0);
        innerController.setI(0.0);
        innerController.setD(0.0);
        innerController.setFF(0.00);
        innerController.setOutputRange(-MAX_SPEED_INNER, MAX_SPEED_INNER);

        innerController.setFeedbackDevice(innerEncoder);

        this.wrist.setInverted(false);
        this.wrist.setIdleMode(IdleMode.kBrake);

        wristController = new PIDController(0.15, 0.0, 0.0);
        wristController.enableContinuousInput(0, 2*Math.PI);
        wristEncoder = this.wrist.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

        // wristController.setP(0.5);
        // wristController.setI(0.0);
        // wristController.setD(0.0);
        // wristController.setFF(0.00);
        // wristController.setOutputRange(-MAX_SPEED_WRIST, MAX_SPEED_WRIST);

        // wristController.setFeedbackDevice(wristEncoder);

        outerEncoder.setPositionConversionFactor(2*Math.PI);
        innerEncoder.setPositionConversionFactor(2*Math.PI);
        wristEncoder.setPositionConversionFactor(2*Math.PI);

        outerArm.burnFlash();
        outerArmFollow.burnFlash();
        innerArm.burnFlash();
        innerArmFollow.burnFlash();
        this.wrist.burnFlash();
    }

    public void incrOut(double amntDeg) {
        double currentPosition = outerEncoder.getPosition();
        setOut(radiansToDegrees(currentPosition + degreesToRadians(amntDeg)));
    }

    public void incrIn(double amntDeg) {
        double currentPosition = innerEncoder.getPosition();
        setIn(radiansToDegrees(currentPosition + degreesToRadians(amntDeg)));
    }

    public void incrWrist(double amntDeg) {
        double currentPosition = wristEncoder.getPosition();
        setWrist(radiansToDegrees(currentPosition + degreesToRadians(amntDeg)));
    }

    public void setOut(double posDeg) {
        setpointOut = degreesToRadians(posDeg);
        Rotation2d ba2g = calcBicepAngleToGround(fromRadians(setpointOut));
        double arbFF = shoulderMinCosineMultiplier * ba2g.getCos() / SHOULDER_TORQUE_TOTAL;
        outerController.setReference(setpointOut, ControlType.kPosition, 0, arbFF, ArbFFUnits.kPercentOut);
        SmartDashboard.putNumber("shoulder set", setpointOut);
    }

    public void setIn(double posDeg) {
        setpointIn = degreesToRadians(posDeg);
        Rotation2d fa2g = calcForearmAngleToGround(bicepAngle, fromRadians(setpointIn));
        double arbFF = -forearmMinCosineMultiplier * fa2g.getCos() / FOREARM_TORQUE_TOTAL;
        System.out.println("famcm " + forearmMinCosineMultiplier);
        System.out.println("fa2g deg " + fa2g.getDegrees());
        System.out.println("fa2g cos " + fa2g.getCos());
        System.out.println("fa t total " + FOREARM_TORQUE_TOTAL);
        System.out.println("arbff " + arbFF);
        innerController.setReference(setpointIn, ControlType.kPosition, 0, arbFF, ArbFFUnits.kPercentOut);
        SmartDashboard.putNumber("elbow set", setpointIn);
    }

    public void setWrist(double posDeg) {
        setpointWrist = degreesToRadians(posDeg);
        // wristController.setReference(setpointWrist, ControlType.kPosition);
        SmartDashboard.putNumber("wrist set", setpointWrist);
    }

    public void speedOut(double spd) {
        double modSpd = MathUtil.clamp(spd, -1, 1);
        outerArm.set(modSpd*MAX_SPEED_OUTER);
    }

    public void speedIn(double spd) {
        double modSpd = MathUtil.clamp(spd, -1, 1);
        innerArm.set(modSpd*MAX_SPEED_INNER);
    }

    public void speedWrist(double spd) {
        double modSpd = MathUtil.clamp(spd, -1, 1);
        wrist.set(modSpd*MAX_SPEED_WRIST);
    }

    public Rotation2d calcBicepAngleToGround(Rotation2d bicepAngle) {
        return bicepAngle.minus(DEG_90);
    }

    public Rotation2d calcForearmAngleToGround(Rotation2d bicepAngle, Rotation2d forearmAngle) {
        return calcBicepAngleToGround(bicepAngle).minus(forearmAngle);
    }

    public Rotation2d calcHandAngleToGround(Rotation2d bicepAngle, Rotation2d forearmAngle, Rotation2d handAngle) {
        return calcForearmAngleToGround(bicepAngle, forearmAngle).plus(handAngle);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("shoulder spd", outerArm.get());
        SmartDashboard.putNumber("elbow spd", innerArm.get());
        SmartDashboard.putNumber("wrist spd", wrist.get());

        bicepAngle = fromRadians(outerEncoder.getPosition());
        forearmAngle = fromRadians(innerEncoder.getPosition());
        handAngle = fromRadians(wristEncoder.getPosition());

        bicepAngleToGround = calcBicepAngleToGround(bicepAngle);
        forearmAngleToGround = calcForearmAngleToGround(bicepAngle, forearmAngle);
        handAngleToGround = calcHandAngleToGround(bicepAngle, forearmAngle, handAngle);

        double wristPow = wristController.calculate(wristEncoder.getPosition(), setpointWrist);
        // wristPow += MAX_SPEED_WRIST * Math.abs(handAngleToGround.getCos());
        Rotation2d wa2g = calcHandAngleToGround(bicepAngle, forearmAngle, fromRadians(setpointWrist));
        double arbFF = wristCosineMultiplier * wa2g.getCos() / WRIST_TORQUE_TOTAL;
        SmartDashboard.putNumber("wrist arbff", arbFF);
        wristPow += arbFF;
        wristPow = MathUtil.clamp(wristPow, -MAX_SPEED_WRIST, MAX_SPEED_WRIST);
        wrist.set(wristPow);

        SmartDashboard.putNumber("wrist pid error", wristController.getPositionError());

        SmartDashboard.putNumber("bicep angle", bicepAngle.getDegrees());
        SmartDashboard.putNumber("bicep angle to ground", bicepAngleToGround.getDegrees());
        SmartDashboard.putNumber("forearm angle", forearmAngle.getDegrees());
        SmartDashboard.putNumber("forearm angle to ground", forearmAngleToGround.getDegrees());
        SmartDashboard.putNumber("hand angle", handAngle.getDegrees());
        SmartDashboard.putNumber("hand angle to ground", handAngleToGround.getDegrees());

        SmartDashboard.putNumber("shoulder cur", bicepAngle.getRadians());
        SmartDashboard.putNumber("elbow cur", forearmAngle.getRadians());
        SmartDashboard.putNumber("wrist cur", handAngle.getRadians());
    }

    public Command defaultCommand(DoubleSupplier outSupplier, DoubleSupplier innerSupplier, DoubleSupplier wristSupplier) {
        return new RunEndCommand(
            () -> {
                this.speedOut(outSupplier.getAsDouble());
                this.speedIn(innerSupplier.getAsDouble());
                this.speedWrist(wristSupplier.getAsDouble());
            },
            (i) -> {
                this.speedOut(0);
                this.speedIn(0);
                this.speedWrist(0);
            },
            this);
    }

    public Command defaultCommand(DoubleSupplier outSupplier, DoubleSupplier innerSupplier) {
        return new RunEndCommand(
            () -> {
                this.speedOut(outSupplier.getAsDouble());
                this.speedIn(innerSupplier.getAsDouble());
            },
            (i) -> {
                this.speedOut(0);
                this.speedIn(0);
            },
            this);
    }
}
