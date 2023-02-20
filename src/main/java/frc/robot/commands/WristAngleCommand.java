package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm.ArmComponent;

public class WristAngleCommand extends CommandBase {
    double degrees;
    ArmComponent wrist;
    
    public WristAngleCommand(ArmComponent wrist, double degrees) {
        this.degrees = degrees;
        this.wrist = wrist;
    }


    @Override
    public void initialize() {
        wrist.setDegrees(degrees);
    }


    @Override
    public boolean isFinished() {
        return true;
    }
}
