package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    CANSparkMax intakeMotor;
    CANSparkMax f_intakeMotor;

    DoubleSolenoid pac;
    DoubleSolenoid f_pac;


    /**
     * Initializes the Intake
     * @param parentMotor The left intake motor
     * @param followerMotor The right intake motor
     * @param pneumaticArmControl The left double solenoid
     * @param followerPneumaticArmControl The right double solenoid
     */
    public Intake(CANSparkMax parentMotor, CANSparkMax followerMotor, DoubleSolenoid pneumaticArmControl, DoubleSolenoid followerPneumaticArmControl) {
        this.intakeMotor = parentMotor;
        this.f_intakeMotor = followerMotor;

        followerMotor.follow(intakeMotor, true); // oppose parent

        pac = pneumaticArmControl;
        f_pac = followerPneumaticArmControl;
    }

    /**
     * Deploys the intake out of the robot.
     */
    public void deploy() {
        pac.set(kForward);
        f_pac.set(kForward);
    }

    /**
     * Retracts the intake back into the robot.
     */
    public void retract() {
        pac.set(kReverse);
        f_pac.set(kReverse);
    }

    /**
     * Depending on the read value of the master solenoid, it will:
     * <ul>
     *   <li> retract if the value is deployed or off
     *   <li> deploy if the value is retracted
     */
    private void toggle() { // temporary commercialization
        DoubleSolenoid.Value masterSolenoidValue = pac.get();
        if(masterSolenoidValue == kForward || masterSolenoidValue == kOff) {
            retract();
            return;
        }
        deploy();
    }

    /**
     * Preferably use this instead, as it is a direct call.
     * @param speed determines how fast the motors spin.
     */
    public void run(double speed) {
        intakeMotor.set(speed);
    }
}