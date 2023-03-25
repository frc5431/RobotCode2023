package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import com.revrobotics.CANSparkMax;

public class Manipulator extends SubsystemBase {

    private final DoubleSolenoid piston;
    private final CANSparkMax manipulatorMotor;

    public static final DoubleSolenoid.Value OPEN_STATE = kReverse;
    public static final DoubleSolenoid.Value CLOSED_STATE = kForward;
    public static boolean isOpen;

    public Manipulator(DoubleSolenoid piston, CANSparkMax manipulatorMotor) {
        this.piston = piston;
        this.manipulatorMotor = manipulatorMotor;
        isOpen = false;
    }

    // open manipulator
    public void open() {
        if (piston.get() == CLOSED_STATE)
            piston.set(OPEN_STATE);
        isOpen = true;
    }

    // close manipulator
    public void close() {
        if (piston.get() == OPEN_STATE)
            piston.set(CLOSED_STATE);
        isOpen = false;
    }

    public void toggle() {
        piston.toggle();
        isOpen = piston.get() == OPEN_STATE;
    }

    public void intakeMotorRun(){
        manipulatorMotor.set(5);
    }

    public void intakeMotorReverse(){
        manipulatorMotor.set(-5);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("manip open", isOpen);
    }

    public boolean isOpen() {
        return isOpen;
    }

    public Command manipCommand(boolean open) {
        return runOnce(() -> {
            if (open) this.open();
            else this.close();
        });
    }

    public Command toggleCommand() {
        return runOnce(this::toggle);
    }
}
