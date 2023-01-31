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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    
    private final CANSparkMax outerArmLeft;
    private final CANSparkMax outerArmRight;
    private final CANSparkMax innerArm;
    private final CANSparkMax innerArmFollow;

    private final SparkMaxPIDController outerLeftController;
    private final AbsoluteEncoder outerLeftEncoder;
    private final SparkMaxPIDController outerRightController;
    private final AbsoluteEncoder outerRightEncoder;

    private final SparkMaxPIDController innerController;
    private final AbsoluteEncoder innerEncoder;

    // private final 

    public static final double MAX_SPEED_OUTER = 0.28;

    private double setpoint = 0;

    public Arm(CANSparkMax outerArmLeft, CANSparkMax outerArmRight, CANSparkMax innerArmLeft, CANSparkMax innerArmRight) {
        this.outerArmLeft = outerArmLeft;
        this.outerArmRight = outerArmRight;
        this.innerArm = innerArmLeft;
        this.innerArmFollow = innerArmRight;

        this.outerArmLeft.setIdleMode(IdleMode.kBrake);
        this.outerArmRight.setIdleMode(IdleMode.kBrake);

        outerLeftController = this.outerArmLeft.getPIDController();
        outerRightController = this.outerArmRight.getPIDController();
        outerLeftEncoder = this.outerArmLeft.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        outerRightEncoder = this.outerArmRight.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

        outerLeftController.setP(0.5);
        // controller.setI(0.0002);
        outerLeftController.setI(0.0);
        outerLeftController.setD(0.0);
        outerLeftController.setFF(0.00);
        outerLeftController.setOutputRange(-1.0, 1.0);

        outerLeftController.setFeedbackDevice(outerLeftEncoder);
        outerRightController.setFeedbackDevice(outerRightEncoder);

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
        // controller.setI(0.0002);
        innerController.setI(0.0);
        innerController.setD(0.0);
        innerController.setFF(0.00);
        innerController.setOutputRange(-0.3, 0.3);

        innerController.setFeedbackDevice(innerEncoder);

        innerArm.burnFlash();
        innerArmFollow.burnFlash();
    }

    public void incr(double amnt) {
        double currentPosition = innerEncoder.getPosition();
        setpoint = currentPosition + amnt;
        innerController.setReference(setpoint, ControlType.kPosition);
        SmartDashboard.putNumber("arm set", setpoint);
    }

    public void set(double pos) {
        setpoint = pos;
        innerController.setReference(setpoint, ControlType.kPosition);
        SmartDashboard.putNumber("arm set", setpoint);
    }

    public void speed(double spd) {
        double modSpd = MathUtil.clamp(spd, -1, 1);
        outerArmLeft.set(modSpd*MAX_SPEED_OUTER);
        outerArmRight.set(-modSpd*MAX_SPEED_OUTER);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("arm pos", innerEncoder.getPosition());
        SmartDashboard.putNumber("out arm lef spd", outerArmLeft.get());
        SmartDashboard.putNumber("out arm rig spd", outerArmRight.get());
    }

    public Command runArmCommand(double amnt) {
        return new StartEndCommand(() -> this.incr(amnt), () -> this.incr(0), this);
    }

    public Command runArmCommand(DoubleSupplier incrSupplier) {
        return new FunctionalCommand(
            () -> {},
            () -> this.incr(incrSupplier.getAsDouble()),
            (i) -> this.incr(0),
            () -> false,
            this);
    }

    public Command speedCmd(DoubleSupplier speedSupplier) {
        return new FunctionalCommand(
            () -> {},
            () -> this.speed(speedSupplier.getAsDouble()),
            (i) -> this.speed(0),
            () -> false,
            this);
    }
}
