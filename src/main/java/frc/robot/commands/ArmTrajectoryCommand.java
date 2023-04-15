package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Systems;

public class ArmTrajectoryCommand extends CommandBase {
    Timer elapsedTime = new Timer();
    private final Trajectory traj;
    private final Systems systems;

    public ArmTrajectoryCommand(Systems systems, Trajectory traj) {
        this.traj = traj;
        this.systems = systems;
        setName("ArmTrajCommand");
        addRequirements(systems.getArm().getAllComponentsForRequirements());
    }

    @Override
    public void initialize() {
        elapsedTime.restart();
    }

    @Override
    public void execute() {
        var state = traj.sample(elapsedTime.get());
        Pose2d set = state.poseMeters;
        systems.getArm().setGoal(set.getTranslation());
        systems.getArm().getWrist().setDegrees(set.getRotation().getDegrees());
    }

    @Override
    public void end(boolean interrupted) {
        elapsedTime.stop();
    }

    @Override
    public boolean isFinished() {
        return elapsedTime.get() > traj.getTotalTimeSeconds();
    }
}
