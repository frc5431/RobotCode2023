package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5431.titan.core.solenoid.SingleSolenoid;

public class Deadwheels extends SubsystemBase {

    private final SingleSolenoid piston;

    private boolean isDeployed;

    public Deadwheels(SingleSolenoid piston) {
        this.piston = piston;
        isDeployed = false;
    }

    public boolean isDeployed() {
        return isDeployed;
    }

    public void deploy() {
        piston.set(true);
        isDeployed = true;
    }

    public void retract() {
        piston.set(false);
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
