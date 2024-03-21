// package frc.robot.commands;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.Pair;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Systems;
// import frc.robot.subsystems.Drivebase;
// import frc.team5431.titan.core.misc.Logger;

// import java.util.function.BooleanSupplier;
// import java.util.function.DoubleSupplier;
// import java.util.function.Supplier;

// public class DriveLockedRotCommand extends Command {
//     private final Drivebase m_drivetrainSubsystem;

//     private final DoubleSupplier m_translationXSupplier;
//     private final DoubleSupplier m_translationYSupplier;
//     private final double gyroAngle;

//     private final BooleanSupplier isManualRotating;

//     private PIDController rotController = new PIDController(6.0, 0.0, 0.0);

//     public DriveLockedRotCommand(Systems systems,
//                                DoubleSupplier translationXSupplier,
//                                DoubleSupplier translationYSupplier,
//                                double gyroAngle,
//                                BooleanSupplier isManualRotating) {
//         this.isManualRotating = isManualRotating;
//         this.m_drivetrainSubsystem = systems.getDrivebase();
//         this.m_translationXSupplier = translationXSupplier;
//         this.m_translationYSupplier = translationYSupplier;
//         this.gyroAngle = gyroAngle;


//         addRequirements(m_drivetrainSubsystem);
//         setName("DefaultDriveCommand");
//     }

//     public DriveLockedRotCommand(Systems systems,
//                                Supplier<Pair<Double, Double>> magThetaSupplier,
//                                double gyroAngle,
//                                BooleanSupplier isManualRotating) {
//         this(
//             systems,
//             () -> magThetaSupplier.get().getFirst() * Math.cos(magThetaSupplier.get().getSecond()),
//             () -> magThetaSupplier.get().getFirst() * Math.sin(magThetaSupplier.get().getSecond()),
//             gyroAngle,
//             isManualRotating
//         );
//     }

//     @Override
//     public void initialize() {
//         Logger.l("Going to rot " + gyroAngle);
//         rotController = new PIDController(0.12, 0.12, 0.012);
//         rotController.setTolerance(1, 2);
//         rotController.enableContinuousInput(0, 360);
//         rotController.setSetpoint(gyroAngle);
//     }

//     @Override
//     public void execute() {
//         double x = m_translationXSupplier.getAsDouble();
//         double y = m_translationYSupplier.getAsDouble();
//         double rot = rotController.calculate(m_drivetrainSubsystem.getGyroscopeRotation().getDegrees());

//         rot = MathUtil.clamp(rot, -Drivebase.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, Drivebase.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);

//         // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
//         m_drivetrainSubsystem.drive(
//                 ChassisSpeeds.fromFieldRelativeSpeeds(
//                         x,
//                         y,
//                         rot,
//                         m_drivetrainSubsystem.getGyroscopeRotation()
//                 )
//         );
//     }

//     @Override
//     public void end(boolean interrupted) {
//         Logger.l("Default drive ending");
//         m_drivetrainSubsystem.stop();
//     }

//     @Override
//     public boolean isFinished() {
//         return rotController.atSetpoint() || isManualRotating.getAsBoolean();
//     }
// }
