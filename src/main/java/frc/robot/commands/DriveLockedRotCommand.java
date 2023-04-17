package frc.robot.commands;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Systems;
import frc.robot.subsystems.Drivebase;
import frc.team5431.titan.core.misc.Logger;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveLockedRotCommand extends CommandBase {
    private final Drivebase m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final double gyroAngle;

    private PIDController rotController = new PIDController(6.0, 0.0, 0.0);

    public DriveLockedRotCommand(Systems systems,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               double gyroAngle) {
        this.m_drivetrainSubsystem = systems.getDrivebase();
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.gyroAngle = gyroAngle;

        addRequirements(m_drivetrainSubsystem);
        setName("DefaultDriveCommand");
    }

    public DriveLockedRotCommand(Systems systems,
                               Supplier<Pair<Double, Double>> magThetaSupplier,
                               double gyroAngle) {
        this(
            systems,
            () -> magThetaSupplier.get().getFirst() * Math.cos(magThetaSupplier.get().getSecond()),
            () -> magThetaSupplier.get().getFirst() * Math.sin(magThetaSupplier.get().getSecond()),
            gyroAngle
        );
    }

    @Override
    public void initialize() {
        Logger.l("Going to rot " + gyroAngle);
        rotController = new PIDController(6.0, 0.0, 0.0);
        rotController.setTolerance(5, 2);
        rotController.setSetpoint(gyroAngle);
    }

    @Override
    public void execute() {
        double x = m_translationXSupplier.getAsDouble();
        double y = m_translationYSupplier.getAsDouble();
        double rot = rotController.calculate(m_drivetrainSubsystem.getGyroscopeRotation().getDegrees());

        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        m_drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        x,
                        y,
                        rot,
                        m_drivetrainSubsystem.getGyroscopeRotation()
                )
        );
    }

    @Override
    public void end(boolean interrupted) {
        Logger.l("Default drive ending");
        m_drivetrainSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return rotController.atSetpoint();
    }
}
