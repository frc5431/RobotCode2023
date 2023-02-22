package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

public class Manipulator {

    private final DoubleSolenoid piston;

    public static final DoubleSolenoid.Value OPEN_STATE = kReverse;
    public static final DoubleSolenoid.Value CLOSED_STATE = kForward;
    private boolean isOpen;

    public Manipulator(DoubleSolenoid piston) {
        this.piston = piston;
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

    public boolean isOpen() {
        return isOpen;
    }

    public DoubleSolenoid.Value getStateFromBool(boolean isOpen) {
        return isOpen ? OPEN_STATE : CLOSED_STATE; 
    }

    public void setState(DoubleSolenoid.Value val) {
        piston.set(val);
    }
}
