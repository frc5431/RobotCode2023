package frc.robot.subsystems;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class Deadwheels {

    private final DoubleSolenoid piston;
    private static final DoubleSolenoid.Value OPEN_STATE = kReverse;
    private static final DoubleSolenoid.Value CLOSED_STATE = kForward;

    private boolean isDeployed;

    public Deadwheels(DoubleSolenoid piston) {
        this.piston = piston;
        isDeployed = false;
    }

    public boolean isDeployed() {
        return isDeployed;
    }

    public void deploy() {
        if (piston.get() == CLOSED_STATE)
            piston.set(OPEN_STATE);
            isDeployed = true;
    }

    public void retract() {
        if (piston.get() == OPEN_STATE)
            piston.set(CLOSED_STATE);
            isDeployed = false;
    }

    public void toggleDeadwheels() {
        piston.toggle();
        isDeployed = !isDeployed;
    }
   
}