// package frc.robot.commands;

// import com.ctre.phoenix6.hardware.Pigeon2;

// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.RobotContainer;
// import frc.robot.Systems;
// import frc.robot.subsystems.Drivebase;
// import frc.team5431.titan.core.leds.Blinkin;
// import frc.team5431.titan.core.misc.Logger;

// /**
//  * Autobalancer code that uses a BangBang Controller intead of PID
//  */
// public class AutobalancerHardcode extends Command {
//     public final Pigeon2 pigy;
//     public final Drivebase drivebase;
//     public final Blinkin leds;

//     public static final double SPEED_VX = 1.0; // m/s
//     public static final double ALLOWED_RETURN_TO_0 = 2.0; // degrees

//     private double farthestGyroFromZero = 0;
//     private boolean finished = false;

//     public AutobalancerHardcode(Systems systems) {
//         this.drivebase = systems.getDrivebase();
//         this.leds = systems.getLeds();
//         this.pigy = drivebase.getGyro();

//         addRequirements(drivebase, leds);
//     }

//     @Override
//     public void initialize() {
//         leds.set(RobotContainer.getPatternFromAlliance(true));

//         farthestGyroFromZero = pigy.getPitch().getValueAsDouble();
//         finished = false;
//         Logger.l("Starting 3015 autobalancer");
//     }

//     @Override
//     public void execute() {
//         double absPitch = Math.abs(pigy.getPitch().getValueAsDouble());
//         double absGyroMax = Math.abs(farthestGyroFromZero);
//         double direction = -1 * Math.copySign(1.0, pigy.getPitch().getValueAsDouble());
//         if (absPitch > absGyroMax) {
//             farthestGyroFromZero = pigy.getPitch().getValueAsDouble();
//             drivebase.drive(new ChassisSpeeds(direction*SPEED_VX, 0, 0));
//         } else if (absPitch > (absGyroMax - ALLOWED_RETURN_TO_0)) {
//             drivebase.drive(new ChassisSpeeds(direction*SPEED_VX, 0, 0));
//         } else {
//             drivebase.stop();
//             finished = true;
//         }
//     }

//     @Override
//     public void end(boolean interrupted) {
//         Logger.l("Ending 3015 autobalancer");
//     }

//     @Override
//     public boolean isFinished() {
//         return finished;
//     }
// }