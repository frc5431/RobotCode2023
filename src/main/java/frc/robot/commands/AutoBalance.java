package frc.robot.commands;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;

public class AutoBalance extends CommandBase{
    public final Pigeon2 pigy;
    public final double desiredDegrees;

    public AutoBalance(double desiredDegrees, Drivebase drivebase){        
           this.desiredDegrees = desiredDegrees;
           this.pigy = drivebase.getGyro();
    }

    @Override
    public void execute(){
        
    }

    @Override
    public boolean isFinished(){
        return false;
    }

}