package frc.robot.commands;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Systems;
import frc.robot.subsystems.Drivebase;
import frc.team5431.titan.core.leds.Blinkin;

/**
 * Autobalancer code that uses a BangBang Controller intead of PID
 */
public class AutoBalancerHardcode extends CommandBase {
    public final Pigeon2 pigy;
    public final Drivebase drivebase;
    public final Blinkin leds;

    public static final double worstPID = 0.15;

    public AutoBalancerHardcode(Systems systems) {
        this.drivebase = systems.getDrivebase();
        this.leds = systems.getLeds();
        this.pigy = drivebase.getGyro();

        addRequirements(drivebase, leds);
    }

    @Override
    public void initialize() {
        leds.set(RobotContainer.getPatternFromAlliance(true));
    }

    @Override
    public void execute() {
        drivebase.drive(new ChassisSpeeds(worstPID * (pigy.getPitch() > 0 ? 1 : -1), 0, 0));
        
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(pigy.getPitch()) < 2.5);
            
        
    }

}