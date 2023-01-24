package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    
    private final CANSparkMax arm1;
    private final CANSparkMax arm2;

    private final SparkMaxPIDController controller;
    private final RelativeEncoder encoder;

    // private final 

    public static final double MAX_SPEED = 0.1;

    public Arm(CANSparkMax armLeft, CANSparkMax armRight) {
        arm1 = armLeft;
        arm2 = armRight;

        arm2.follow(arm1, true);
        arm1.setIdleMode(IdleMode.kBrake);
        arm2.setIdleMode(IdleMode.kBrake);

        controller = arm1.getPIDController();
        encoder = arm1.getEncoder();

        controller.setP(0.1);
        controller.setI(0.0);
        controller.setD(0.0);
        controller.setFF(0.0);
        controller.setOutputRange(-1, 1);

        arm1.burnFlash();
        arm2.burnFlash();
    }

    public void set(double amnt) {
        // double actualSpeed = MathUtil.clamp(speed, -1, 1) * MAX_SPEED;
        // arm1.set(actualSpeed);
        double currentPosition = encoder.getPosition();
        controller.setReference(currentPosition + amnt, ControlType.kPosition);
    }

    @Override
    public void periodic() {
        // controller.setReference(0, ControlType.kPosition);
        
    }

    public Command runArmCommand(double speed) {
        return new StartEndCommand(() -> this.set(speed), () -> this.set(0), this);
    }

    public Command runArmCommand(DoubleSupplier speedSupplier) {
        return new FunctionalCommand(
            () -> {},
            () -> this.set(speedSupplier.getAsDouble()),
            (i) -> this.set(0),
            () -> false,
            this);
    }
}
