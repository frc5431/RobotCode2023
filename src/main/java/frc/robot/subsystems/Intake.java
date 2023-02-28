package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    public static final double DEFAULT_SPEED = 1.0;
    
    private final CANSparkMax intakeMotor;
    private final CANSparkMax intakeMotorFollow;

    private final DoubleSolenoid piston;
    private static final DoubleSolenoid.Value DOWN_STATE = kReverse;
    private static final DoubleSolenoid.Value UP_STATE = kForward;

    private boolean isDeployed;

    /**
     * Initializes the Intake
     * @param intakeLeft The left intake motor
     * @param intakeRight The right intake motor
     * @param deployPiston The left double solenoid
     * @param followerPneumaticArmControl The right double solenoid
     */
    public Intake(CANSparkMax intakeLeft, CANSparkMax intakeRight, DoubleSolenoid deployPiston) {
        intakeMotor = intakeLeft;
        intakeMotorFollow = intakeRight;

        intakeMotor.setInverted(false);
        intakeMotorFollow.follow(intakeMotor, true); // oppose parent

        intakeMotor.setIdleMode(IdleMode.kCoast);
        intakeMotorFollow.setIdleMode(IdleMode.kCoast);

        intakeMotor.setSmartCurrentLimit(20);
        intakeMotorFollow.setSmartCurrentLimit(20);

        intakeMotor.burnFlash();
        intakeMotorFollow.burnFlash();

        piston = deployPiston;

        isDeployed = false;
    }

    public boolean isDeployed() {
        return isDeployed;
    }

    /**
     * Deploys the intake out of the robot.
     */
    public void deploy() {
        if (piston.get() == UP_STATE)
            piston.set(DOWN_STATE);
        isDeployed = true;
    }

    /**
     * Retracts the intake back into the robot.
     */
    public void retract() {
        if (piston.get() == DOWN_STATE)
            piston.set(UP_STATE);
        isDeployed = false;
    }

    /**
     * Toggle the intake deployed status
     */
    public void toggle() {
        piston.toggle();
        isDeployed = !isDeployed;
    }

    /**
     * Preferably use this instead, as it is a direct call.
     * @param speed determines how fast the motors spin.
     */
    public void set(double speed) {
        intakeMotor.set(speed);
    }

    public Command runIntakeCommand(boolean reverse) {
        return runIntakeCommand(reverse ? -DEFAULT_SPEED : DEFAULT_SPEED);
    }

    public Command runIntakeCommand(double speed) {
        return new StartEndCommand(() -> this.set(speed), () -> this.set(0), this);
    }

    public Command floorIntakeCommand() {
        return runOnce(this::deploy).andThen(runIntakeCommand(false));
    }

    public Command intakeStow() {
        return runOnce(() -> {
            this.set(0);
            this.retract();
        });
    }

    // by special request of brian
    public Command runSameDirection(boolean reverse) {
        return new StartEndCommand(() -> {
            this.intakeMotorFollow.follow(this.intakeMotor, false);
            this.set(reverse ? -DEFAULT_SPEED : DEFAULT_SPEED);
        }, () -> {
            this.set(0);
            this.intakeMotorFollow.follow(this.intakeMotor, true);
        }, this);
    }
}
