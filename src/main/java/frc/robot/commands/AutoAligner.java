package frc.robot.commands;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;


public class AutoAligner extends CommandBase{
    public final Pigeon2 pigy;
    public final Drivebase drivebase;
    public PIDController pid = new PIDController(0, 0, 0);
    public ChassisSpeeds cs = new ChassisSpeeds(0, 0, 0);
    
    public AutoAligner(Drivebase drivebase){
           this.drivebase = drivebase;
           this.pigy = drivebase.getGyro();
           pid.setTolerance(2.5);
    }       

    @Override
    public void initialize(){
        boolean robotIsBackwards = Math.abs(180 - pigy.getYaw()) <= 1;
        double direction = robotIsBackwards ? 1 : -1; // TODO direction is not used? // yeah i forgot why
    }

    @Override
    public void execute(){
        cs.vxMetersPerSecond = pid.calculate(pigy.getPitch(), 0);
        drivebase.drive(cs);  
    }

    @Override
    public boolean isFinished(){
        return pid.atSetpoint();
    }

}