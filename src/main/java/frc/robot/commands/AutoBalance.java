package frc.robot.commands;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;

public class AutoBalance extends CommandBase{
    public final Pigeon2 pigy;
    public final double desiredDegrees;
    public final Drivebase drivebase;
    

    public AutoBalance(double desiredDegrees, Drivebase drivebase){        
           this.desiredDegrees = desiredDegrees;
           this.pigy = drivebase.getGyro();
           this.drivebase = drivebase;
    }

    @Override
    public void execute(){
        double setpoint = desiredDegrees;
        double processVariable = pigy.getPitch();
        double error = setpoint - processVariable;
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.1, 0.0, 0.0);

        if (Math.abs(error) > 2.5) {
            double kP = 1.0;
            drivebase.drive(chassisSpeeds);
        } else if(Math.abs(error) < -2.5) {
            drivebase.drive(chassisSpeeds);
        }
    }

    @Override
    public boolean isFinished(){
        return true;
    }

}