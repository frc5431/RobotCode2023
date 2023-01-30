package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;




public class Manipulator {

    public boolean extended = false;
    public static final int manipForwardChannel = 4;
    public static final int manipReverseChannel = 8;

    DoubleSolenoid manipPisston = new DoubleSolenoid(1, PneumaticsModuleType.REVPH, manipForwardChannel, manipReverseChannel);

    //extends manipulator pistion using 1 double solinoide 
    public void manipulatorRetract() {
        if(extended){manipPisston.set(DoubleSolenoid.Value.kReverse);
            extended = false;
        }
    }
   
    //retracts manipulator pistion using 1 double solinoide 
    public void manipulatorDeploy() {
        if(!extended){manipPisston.set(DoubleSolenoid.Value.kForward); 
            extended = true;
        }
    }

}
