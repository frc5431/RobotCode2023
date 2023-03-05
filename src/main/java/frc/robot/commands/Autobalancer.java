package frc.robot.commands;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Systems;
import frc.robot.subsystems.Drivebase;
import frc.team5431.titan.core.leds.Blinkin;
import frc.team5431.titan.core.robot.MotionMagic;

public class Autobalancer extends CommandBase {
    public final Pigeon2 pigy;
    public final Drivebase drivebase;
    public final Blinkin leds;

    public final MotionMagic PID_NORMAL = new MotionMagic(0.035, 0, 0.01, 0.0);
    public final MotionMagic PID_BLUE   = new MotionMagic(0.035, 0, 0.01, 0.0);

    private PIDController pid = new PIDController(PID_NORMAL.p(), PID_NORMAL.i(), PID_NORMAL.d());
    public ChassisSpeeds cs = new ChassisSpeeds(0, 0, 0);

    public Autobalancer(Systems systems) {
        this.drivebase = systems.getDrivebase();
        this.leds = systems.getLeds();
        this.pigy = drivebase.getGyro();

        addRequirements(drivebase, leds);
    }

    @Override
    public void initialize() {
        leds.set(RobotContainer.getPatternFromAlliance(true));

        if (DriverStation.getAlliance() == Alliance.Blue) {
            pid = new PIDController(PID_BLUE.p(), PID_BLUE.i(), PID_BLUE.d());
        } else {
            pid = new PIDController(PID_NORMAL.p(), PID_NORMAL.i(), PID_NORMAL.d());
        }

        pid.setTolerance(2.5, 0.08);
    }

    @Override
    public void execute() {
        cs.vxMetersPerSecond = pid.calculate(pigy.getPitch(), 0);
        drivebase.drive(cs);
    }

    @Override
    public boolean isFinished() {
        return pid.atSetpoint();
    }

}