package frc.robot.subsystems;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.Command;

public class Deadwheels {

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



    
//  new RunCommand(() -> piston.deploy());

//  new RunCommand(() -> piston.toggle());

//  new RunCommand(() -> piston.retract());


}
