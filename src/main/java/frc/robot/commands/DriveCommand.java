// package frc.robot.commands;

// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Systems;
// import frc.robot.subsystems.Drivebase;
// import frc.team5431.titan.core.misc.Logger;

// public class DriveCommand extends Command {
//     private final Drivebase drivebase;
//     private final ChassisSpeeds speeds;
//     private final boolean robotOriented;

//     public DriveCommand(Systems systems, ChassisSpeeds speeds, boolean robotOriented) {
//         this.drivebase = systems.getDrivebase();
//         this.speeds = speeds;
//         this.robotOriented = robotOriented;

//         addRequirements(drivebase);
//     }
//     public DriveCommand(Systems systems, ChassisSpeeds speeds) {
//         this(systems, speeds, false);
//     }

//     @Override
//     public void initialize() {
//         Logger.l("Starting auto drive command");
//     }

//     @Override
//     public void execute() {
//         if (robotOriented) {
//             drivebase.driveRaw(speeds);
//         } else {
//             drivebase.driveRaw((ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drivebase.getGyroscopeRotation())));
//         }
//     }

//     @Override
//     public void end(boolean interrupted) {
//         Logger.l("Ending auto drive command");
//         drivebase.stop();
//     }
// }
