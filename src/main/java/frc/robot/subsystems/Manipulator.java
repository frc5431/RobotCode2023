package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import com.revrobotics.CANSparkMax;

public class Manipulator extends SubsystemBase {
    private static final double INTAKE_SPEED = 0.3;
    private final CANSparkMax motor;

    public static boolean isRunning;

    public Manipulator(CANSparkMax motor) {
        this.motor = motor;
        isRunning = false;
    }

    public void intake() {
        motor.set(INTAKE_SPEED);
        isRunning = true;
    }

    public void outtake() {
        motor.set(-INTAKE_SPEED);
        isRunning = true;
    }

    public void stop() {
        motor.set(0);
        isRunning = false;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("manip running", isRunning);
    }

    public boolean isRunning() {
        return isRunning;
    }

    public Command manipCommand(boolean open) {
        return runOnce(() -> {
            if (open) this.intake();
            else this.outtake();
        });
    }
}
