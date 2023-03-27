package frc.robot.commands;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Systems;
import frc.robot.subsystems.Drivebase;
import frc.team5431.titan.core.leds.Blinkin;

public class AutoAlignerBangBang extends CommandBase {
    public final Pigeon2 pigy;
    public final Drivebase drivebase;
    public final Blinkin leds;

    public BangBangController bangin = new BangBangController(2.5);
    public ChassisSpeeds cs = new ChassisSpeeds(0, 0, 0);

    public AutoAlignerBangBang(Systems systems) {
        this.drivebase = systems.getDrivebase();
        this.leds = systems.getLeds();
        this.pigy = drivebase.getGyro();

        addRequirements(drivebase, leds);
    }

    @Override
    public void initialize() {
        leds.set(RobotContainer.getPatternFromAlliance(true));

        if (DriverStation.getAlliance() == Alliance.Blue) {
            pigy.setYaw(180);
        }
    }

    @Override
    public void execute() {
        cs.vxMetersPerSecond = bangin.calculate(pigy.getPitch(), 0);
        drivebase.drive(cs);
    }

    @Override
    public boolean isFinished() {
        return bangin.atSetpoint();
    }

}