package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class Manipulator {
    
    private Manipulator manipulator;
    private DoubleSolenoid manipPisston;


    manipPisston = new DoubleSolenoid(1,PneumaticsModuleType.REVPH, manipForwardChannel, manipReverseChannel);
    manipulator = new Manipulator(manipPisston);

    public void manipulatorPistonRun() {

    }

}
