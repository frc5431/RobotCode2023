package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.RunEndCommand;

import static edu.wpi.first.math.geometry.Rotation2d.fromDegrees;
import static edu.wpi.first.math.geometry.Rotation2d.fromRadians;

import static edu.wpi.first.math.util.Units.degreesToRadians;

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
    private final SparkMaxPIDController wristController;
    private final AbsoluteEncoder wristEncoder;

    public static final double MAX_SPEED_OUTER = 0.35;
    public static final double MAX_SPEED_INNER = 0.4;
    public static final double MAX_SPEED_WRIST = 0.50;

    private double setpointOut = 0;
    private double setpointIn = 0;
    private double setpointWrist = 0;

    private static final Rotation2d DEG_90 = fromDegrees(90);

    // measured
    private Rotation2d bicepAngle = new Rotation2d();
    private Rotation2d bicepAngleToGround = new Rotation2d();
    private Rotation2d forearmAngle = new Rotation2d();
    private Rotation2d forearmAngleToGround = new Rotation2d();
    private Rotation2d handAngle = new Rotation2d();
    private Rotation2d handAngleToGround = new Rotation2d();

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

        outerArm.setInverted(true);
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

        innerController.setP(0.5);
        innerController.setI(0.0);
        innerController.setD(0.0);
        innerController.setFF(0.00);
        innerController.setOutputRange(-MAX_SPEED_INNER, MAX_SPEED_INNER);

        innerController.setFeedbackDevice(innerEncoder);

        this.wrist.setInverted(false);
        this.wrist.setIdleMode(IdleMode.kBrake);

        wristController = this.wrist.getPIDController();
        wristEncoder = this.wrist.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

        wristController.setP(0.5);
        wristController.setI(0.0);
        wristController.setD(0.0);
        wristController.setFF(0.00);
        wristController.setOutputRange(-MAX_SPEED_WRIST, MAX_SPEED_WRIST);

        wristController.setFeedbackDevice(wristEncoder);

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
        setpointOut = currentPosition + degreesToRadians(amntDeg);
        outerController.setReference(setpointOut, ControlType.kPosition);
        SmartDashboard.putNumber("shoulder set", setpointOut);
    }

    public void incrIn(double amntDeg) {
        double currentPosition = innerEncoder.getPosition();
        setpointIn = currentPosition + degreesToRadians(amntDeg);
        innerController.setReference(setpointIn, ControlType.kPosition);
        SmartDashboard.putNumber("elbow set", setpointIn);
    }

    public void incrWrist(double amntDeg) {
        double currentPosition = wristEncoder.getPosition();
        setpointWrist = currentPosition + degreesToRadians(amntDeg);
        wristController.setReference(setpointWrist, ControlType.kPosition);
        SmartDashboard.putNumber("wrist set", setpointWrist);
    }

    public void setOut(double posDeg) {
        setpointOut = degreesToRadians(posDeg);
        outerController.setReference(setpointOut, ControlType.kPosition);
        SmartDashboard.putNumber("shoulder set", setpointOut);
    }

    public void setIn(double posDeg) {
        setpointIn = degreesToRadians(posDeg);
        innerController.setReference(setpointIn, ControlType.kPosition);
        SmartDashboard.putNumber("elbow set", setpointIn);
    }

    public void setWrist(double posDeg) {
        setpointWrist = degreesToRadians(posDeg);
        wristController.setReference(setpointWrist, ControlType.kPosition);
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

    @Override
    public void periodic() {
        SmartDashboard.putNumber("shoulder spd", outerArm.get());
        SmartDashboard.putNumber("elbow spd", innerArm.get());
        SmartDashboard.putNumber("wrist spd", wrist.get());

        bicepAngle = fromRadians(outerEncoder.getPosition());
        forearmAngle = fromRadians(innerEncoder.getPosition());
        handAngle = fromRadians(wristEncoder.getPosition());

        bicepAngleToGround = bicepAngle.minus(DEG_90);
        forearmAngleToGround = bicepAngleToGround.minus(forearmAngle);
        handAngleToGround = forearmAngleToGround.plus(handAngle);

        SmartDashboard.putNumber("bicep angle", bicepAngle.getDegrees());
        SmartDashboard.putNumber("bicep angle to ground", bicepAngleToGround.getDegrees());
        SmartDashboard.putNumber("forearm angle", forearmAngle.getDegrees());
        SmartDashboard.putNumber("forearm angle to ground", forearmAngleToGround.getDegrees());
        SmartDashboard.putNumber("hand angle", handAngle.getDegrees());
        SmartDashboard.putNumber("hand angle to ground", handAngleToGround.getDegrees());
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
}
