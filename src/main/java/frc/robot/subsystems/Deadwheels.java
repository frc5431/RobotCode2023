package frc.robot.subsystems;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Deadwheels extends SubsystemBase {

    private final DoubleSolenoid piston;
    private static final DoubleSolenoid.Value DOWN_STATE = kReverse;
    private static final DoubleSolenoid.Value UP_STATE = kForward;

    private boolean isDeployed;

    public Deadwheels(DoubleSolenoid piston) {
        this.piston = piston;
        isDeployed = false;
    }

    public boolean isDeployed() {
        return isDeployed;
    }

    public void deploy() {
        if (piston.get() == UP_STATE)
            piston.set(DOWN_STATE);
        isDeployed = true;
    }

    public void retract() {
        if (piston.get() == DOWN_STATE)
            piston.set(UP_STATE);
        isDeployed = false;
    }

    public void toggle() {
        piston.toggle();
        isDeployed = !isDeployed;
    }

    public Command deadwheelsCommand(boolean deploy) {
        return runOnce(() -> {
            if (deploy) this.deploy();
            else this.retract();
        });
    }
}
