package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Drivebase extends SubsystemBase {

    public abstract void resetOdometry(Pose2d pose);

    public abstract void zeroGyroscope();
    public abstract void resetGyroAt(double yaw);
    public abstract Rotation2d getGyroscopeRotation();
    public abstract WPI_Pigeon2 getGyro();

    public abstract void drive(ChassisSpeeds chassisSpeeds);
    public abstract void stop();

}
