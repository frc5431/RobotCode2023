package frc.robot.commands;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Systems;
import frc.robot.subsystems.Drivebase;
import frc.team5431.titan.core.leds.Blinkin;

/**
 * Autobalancer code that uses a BangBang Controller intead of PID
 */
public class AutobalancerBangBang extends CommandBase {
    public final Pigeon2 pigy;
    public final Drivebase drivebase;
    public final Blinkin leds;

    public BangBangController bangin = new BangBangController(2.5);
    public ChassisSpeeds cs = new ChassisSpeeds(0, 0, 0);

    public AutobalancerBangBang(Systems systems) {
        this.drivebase = systems.getDrivebase();
        this.leds = systems.getLeds();
        this.pigy = drivebase.getGyro();

        addRequirements(drivebase, leds);
    }

    @Override
    public void initialize() {
        leds.set(RobotContainer.getPatternFromAlliance(true));
        bangin = new BangBangController(2.5);
    }

    @Override
    public void execute() {
        cs.vxMetersPerSecond = (pigy.getPitch() > 0 ? -1 : 1) * bangin.calculate(Math.abs(pigy.getPitch()), 0);
        drivebase.drive(cs);
    }

    @Override
    public boolean isFinished() {
        return bangin.atSetpoint();
    }

}