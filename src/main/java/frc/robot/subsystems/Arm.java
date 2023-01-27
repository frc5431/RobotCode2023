package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    
    private final CANSparkMax arm1;
    private final CANSparkMax arm2;

    private final SparkMaxPIDController controller;
    private final AbsoluteEncoder encoder;

    // private final 

    // public static final double MAX_SPEED = 0.1;

    public Arm(CANSparkMax armLeft, CANSparkMax armRight) {
        arm1 = armLeft;
        arm2 = armRight;

        arm2.follow(arm1, true);
        arm1.setIdleMode(IdleMode.kBrake);
        arm2.setIdleMode(IdleMode.kBrake);

        arm1.enableSoftLimit(SoftLimitDirection.kForward, true);
        arm1.enableSoftLimit(SoftLimitDirection.kReverse, true);

        arm1.setSoftLimit(SoftLimitDirection.kForward, 0.80f);
        arm1.setSoftLimit(SoftLimitDirection.kReverse, 0.20f);

        controller = arm1.getPIDController();
        encoder = arm1.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

        controller.setP(0.5);
        controller.setI(0.0002);
        controller.setD(0.0);
        controller.setFF(0.00);
        controller.setOutputRange(-0.3, 0.3);

        controller.setFeedbackDevice(encoder);

        arm1.burnFlash();
        arm2.burnFlash();
    }

    public void incr(double amnt) {
        // double actualSpeed = MathUtil.clamp(speed, -1, 1) * MAX_SPEED;
        // arm1.set(actualSpeed);
        double currentPosition = encoder.getPosition();
        controller.setReference(currentPosition + amnt, ControlType.kPosition);
        DriverStation.reportWarning("setting to pos "+(currentPosition+amnt), false);
    }

    public void set(double pos) {
        controller.setReference(pos, ControlType.kPosition);
        DriverStation.reportWarning("setting to pos "+pos, false);
    }

    @Override
    public void periodic() {
        // controller.setReference(0, ControlType.kPosition);
        SmartDashboard.putNumber("arm pos", encoder.getPosition()+Math.random());
        DriverStation.reportWarning(""+encoder.getPosition(), false);
    }

    public Command runArmCommand(double amnt) {
        return new StartEndCommand(() -> this.incr(amnt), () -> this.incr(0), this);
    }

    public Command runArmCommand(DoubleSupplier speedSupplier) {
        return new FunctionalCommand(
            () -> {},
            () -> this.incr(speedSupplier.getAsDouble()),
            (i) -> this.incr(0),
            () -> false,
            this);
    }
}
