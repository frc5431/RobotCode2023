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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.RunEndCommand;
import frc.robot.util.InverseKinematicsSolver;
import frc.team5431.titan.core.misc.Calc;

import static edu.wpi.first.math.geometry.Rotation2d.fromDegrees;
import static edu.wpi.first.math.geometry.Rotation2d.fromRadians;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.math.util.Units.radiansToDegrees;

public class Arm extends SubsystemBase {
    
    protected final CANSparkMax outerArm;
    protected final CANSparkMax outerArmFollow;
    protected final CANSparkMax innerArm;
    protected final CANSparkMax innerArmFollow;

    protected final SparkMaxPIDController outerController;
    protected final AbsoluteEncoder outerEncoder;

    protected final SparkMaxPIDController innerController;
    protected final AbsoluteEncoder innerEncoder;

    protected final CANSparkMax wrist;
    protected final PIDController wristController;
    protected final AbsoluteEncoder wristEncoder;
    
    public static final double MAX_SPEED_OUTER = 0.3; // 0.22 to hold at horz
    public static final double MAX_SPEED_INNER = 0.25;  // 0.18 to hold at horz
    public static final double MAX_SPEED_WRIST = 0.1;  // 0.064 to hold at horz

    protected double setpointOut = 0;
    protected double setpointIn = 0;
    protected double setpointWrist = 0;

    protected static final Rotation2d DEG_90 = fromDegrees(90);
    protected static final double TORQUE_NM_NEO = 2.6;

    public static final double SHOULDER_TORQUE_TOTAL = TORQUE_NM_NEO * 60 * 2;
    public static final double FOREARM_TORQUE_TOTAL = TORQUE_NM_NEO * 5 * 84/20 * 2;
    public static final double WRIST_TORQUE_TOTAL = TORQUE_NM_NEO * 9;

    // measured
    protected Rotation2d bicepAngle = new Rotation2d();
    protected Rotation2d bicepAngleToGround = new Rotation2d();
    protected Rotation2d forearmAngle = new Rotation2d();
    protected Rotation2d forearmAngleToGround = new Rotation2d();
    protected Rotation2d handAngle = new Rotation2d();
    protected Rotation2d handAngleToGround = new Rotation2d();

    protected final double SETPOINT_POSITION_TOLERANCE = Units.degreesToRadians(1);
    protected final double SETPOINT_VELOCITY_TOLERANCE = Units.degreesToRadians(5);

    protected InverseKinematicsSolver solver = new InverseKinematicsSolver(Units.inchesToMeters(34), Units.inchesToMeters(26));

    protected Translation2d goalPose = new Translation2d(Units.inchesToMeters(5), -Units.inchesToMeters(30));

    // ((mass (kg) * acceleration (m/s/s)) (N) * distance of center of mass from pivot (m)) (Nm)
    public static final double shoulderCosineMultiplierNoCOM =
        8.78 * 9.81;

    public static final double shoulderMinCOMMeters =
        Units.inchesToMeters(18.624);

    public static final double shoulderMaxCOMMeters =
        Units.inchesToMeters(31.239);
    
    public static final double elbowCosineMultiplierNoCOM =
        3.7 * 9.81;
    
    public static final double elbowMinCOMMeters =
        Units.inchesToMeters(19.122);
    
    public static final double elbowMaxCOMMeters =
        Units.inchesToMeters(22.93);

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
        outerController.setPositionPIDWrappingEnabled(true);
        outerController.setPositionPIDWrappingMinInput(0);
        outerController.setPositionPIDWrappingMaxInput(2*Math.PI);

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
        innerController.setPositionPIDWrappingEnabled(true);
        innerController.setPositionPIDWrappingMinInput(0);
        innerController.setPositionPIDWrappingMaxInput(2*Math.PI);

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

        outerEncoder.setVelocityConversionFactor(2*Math.PI);
        innerEncoder.setVelocityConversionFactor(2*Math.PI);
        wristEncoder.setVelocityConversionFactor(2*Math.PI);

        outerArm.burnFlash();
        outerArmFollow.burnFlash();
        innerArm.burnFlash();
        innerArmFollow.burnFlash();
        this.wrist.burnFlash();
    }

    public void setGoal(Translation2d v) {
        goalPose = v;
    }

    public Translation2d getGoal() {
        return goalPose;
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
        double arbFF = shoulderCosineMultiplierNoCOM * getCOMBicepMeters() * ba2g.getCos() / SHOULDER_TORQUE_TOTAL;
        outerController.setReference(setpointOut, ControlType.kPosition, 0, arbFF, ArbFFUnits.kPercentOut);
        SmartDashboard.putNumber("shoulder set", setpointOut);
        SmartDashboard.putNumber("shoulder arbff", arbFF);
    }

    public void setIn(double posDeg) {
        setpointIn = degreesToRadians(posDeg);
        Rotation2d fa2g = calcForearmAngleToGround(bicepAngle, fromRadians(setpointIn));
        double arbFF = -elbowCosineMultiplierNoCOM * getCOMForearmMeters() * fa2g.getCos() / FOREARM_TORQUE_TOTAL;
        // System.out.println("famcm " + forearmMinCosineMultiplier);
        // System.out.println("fa2g deg " + fa2g.getDegrees());
        // System.out.println("fa2g cos " + fa2g.getCos());
        // System.out.println("fa t total " + FOREARM_TORQUE_TOTAL);
        // System.out.println("arbff " + arbFF);
        innerController.setReference(setpointIn, ControlType.kPosition, 0, arbFF, ArbFFUnits.kPercentOut);
        SmartDashboard.putNumber("elbow set", setpointIn);
        SmartDashboard.putNumber("elbow arbff", arbFF);
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

    public double getCOMBicepMeters() {
        double mapped = -innerEncoder.getPosition();
        return Calc.map(Math.cos(mapped), 1, -1, shoulderMaxCOMMeters, shoulderMinCOMMeters);
    }

    public double getCOMForearmMeters() {
        double mapped = wristEncoder.getPosition();
        return Calc.map(Math.cos(mapped), 1, -1, elbowMaxCOMMeters, elbowMinCOMMeters);
    }

    public boolean shoulderAtSetpoint() {
        return (outerEncoder.getPosition() - setpointOut) < SETPOINT_POSITION_TOLERANCE 
            && outerEncoder.getVelocity() < SETPOINT_VELOCITY_TOLERANCE;
    }

    public boolean elbowAtSetpoint() {
        return (innerEncoder.getPosition() - setpointIn) < SETPOINT_POSITION_TOLERANCE
            && innerEncoder.getVelocity() < SETPOINT_VELOCITY_TOLERANCE;
    }

    public boolean wristAtSetpoint() {
        return (wristEncoder.getPosition() - setpointWrist) < SETPOINT_POSITION_TOLERANCE
            && wristEncoder.getVelocity() < SETPOINT_VELOCITY_TOLERANCE;
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

        solveKinematics(goalPose);
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

    public void solveKinematics(Translation2d goal) {
        var ik = solver.solveForPosition(goal);
        if (!Double.isNaN(ik.getOuter()))
            setOut(ik.getOuter());
        if (!Double.isNaN(ik.getInner()))
            setIn(ik.getInner());

        SmartDashboard.putNumber("Goal X", Units.metersToInches(goal.getX()));
        SmartDashboard.putNumber("Goal Y", Units.metersToInches(goal.getY()));
        SmartDashboard.putNumber("InvKin Out", ik.getOuter());
        SmartDashboard.putNumber("InvKin In",  ik.getInner());
        // System.out.println(String.format("Pos: %s, %s. Produced angles: %s, %s", goal.getX(), goal.getY(), ik.getOuter(), ik.getInner()));
    }
}
