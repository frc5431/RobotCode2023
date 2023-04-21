package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import com.revrobotics.CANSparkMax;

public class Manipulator extends SubsystemBase {
    private static final double INTAKE_SPEED = 0.8;
    private final CANSparkMax motor;

    public static enum MotorDirection { FORWARD, REVERSE, NONE; }
    public static enum GamePiece { CONE, CUBE, NONE; }

    public MotorDirection currentDirection;
    public GamePiece heldGamePiece;

    public Manipulator(CANSparkMax motor) {
        motor.restoreFactoryDefaults();
        motor.clearFaults();
        motor.setInverted(true);
        motor.setSmartCurrentLimit(40, 30);
        motor.setOpenLoopRampRate(0.2);
        motor.burnFlash();
        motor.set(0);

        currentDirection = MotorDirection.NONE;
        heldGamePiece = GamePiece.NONE;

        this.motor = motor;
    }

    public void setPositive() {
        motor.set(INTAKE_SPEED);
        currentDirection = MotorDirection.FORWARD;
    }

    public void setNegative() {
        motor.set(-INTAKE_SPEED);
        currentDirection = MotorDirection.REVERSE;
    }

    public void stop() {
        motor.set(0);
        currentDirection = MotorDirection.NONE;
    }

    @Override
    public void periodic() {
        double current = motor.getOutputCurrent();
        SmartDashboard.putNumber("manip amps", current);
        SmartDashboard.putString("cur gp", heldGamePiece.name());
        SmartDashboard.putString("cur md", currentDirection.name());

        if (current > 35) {
            // Stalling, have game piece
            if (currentDirection == MotorDirection.FORWARD) {
                heldGamePiece = GamePiece.CUBE;
            } else if (currentDirection == MotorDirection.REVERSE) {
                heldGamePiece = GamePiece.CONE;
            } else {
                // wtf how are we reaching 40A without motor direction
            }
        } else if (currentDirection != MotorDirection.NONE) {
            heldGamePiece = GamePiece.NONE;
        }
    }

    public GamePiece getHeldGamePiece() {
        return heldGamePiece;
    }

    public CANSparkMax getMotor() {
        return motor;
    }

    public Command manipRunOnceCommand(GamePiece piece, boolean intake) {
        return runOnce(() -> {
            switch (piece) {
                case CONE:
                    if (intake) setNegative();
                    else setPositive();
                    break;
                case CUBE:
                    if (intake) setPositive();
                    else setNegative();
                    break;
                default:
                    stop();
                    break;
            }
        });
    }

    public Command manipRunCommand(GamePiece piece, boolean intake) {
        return new StartEndCommand(() -> {
            switch (piece) {
                case CONE:
                    if (intake) setNegative();
                    else setPositive();
                    break;
                case CUBE:
                    if (intake) setPositive();
                    else setNegative();
                    break;
                default:
                    stop();
                    break;
            }
        }, () -> stop(), this).withName(piece.name() + (intake ? " intake" : " outtake"));
    }
}
