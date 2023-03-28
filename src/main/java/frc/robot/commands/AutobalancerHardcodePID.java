package frc.robot.commands;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Systems;
import frc.robot.subsystems.Drivebase;
import frc.team5431.titan.core.leds.Blinkin;
import frc.team5431.titan.core.misc.Logger;

/**
 * Autobalancer code that uses a BangBang Controller intead of PID
 */
public class AutobalancerHardcodePID extends CommandBase {
    public final Pigeon2 pigy;
    public final Drivebase drivebase;
    public final Blinkin leds;

    public static final double SPEED_VX = 1.0; // m/s
    public static final double ALLOWED_RETURN_TO_0 = 2.5; // degrees

    private double farthestGyroFromZero = 0;
    private boolean startPID = false;
    private boolean finished = false;

    public PIDController pid = new PIDController(0.03, 0, 0.01);

    public AutobalancerHardcodePID(Systems systems) {
        this.drivebase = systems.getDrivebase();
        this.leds = systems.getLeds();
        this.pigy = drivebase.getGyro();

        addRequirements(drivebase, leds);
    }

    @Override
    public void initialize() {
        leds.set(RobotContainer.getPatternFromAlliance(true));

        farthestGyroFromZero = pigy.getPitch();
        startPID = false;
        finished = false;
        Logger.l("Starting 3015 autobalancer");

        pid = new PIDController(0.03, 0, 0.01);

        pid.setTolerance(2.5, 0.08);
    }

    @Override
    public void execute() {
        if (startPID) {
            drivebase.drive(new ChassisSpeeds(pid.calculate(pigy.getPitch(), 5), 0, 0));
            return;
        }
        double absPitch = Math.abs(pigy.getPitch());
        double absGyroMax = Math.abs(farthestGyroFromZero);
        double direction = -1 * Math.copySign(1.0, pigy.getPitch());
        if (absPitch > absGyroMax) {
            farthestGyroFromZero = pigy.getPitch();
            drivebase.drive(new ChassisSpeeds(direction*SPEED_VX, 0, 0));
        } else if (absPitch > (absGyroMax - ALLOWED_RETURN_TO_0)) {
            drivebase.drive(new ChassisSpeeds(direction*SPEED_VX, 0, 0));
        } else {
            drivebase.stop();
            startPID = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        Logger.l("Ending 3015 autobalancer");
    }

    @Override
    public boolean isFinished() {
        return pid.atSetpoint() || finished;
    }
}