package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Systems;
import frc.robot.util.PresetPosition;

public class ArmTrajectoryCommand extends CommandBase {
    Timer elapsedTime = new Timer();
    private Trajectory traj;
    private final List<PresetPosition> setPos;
    private final PresetPosition end;
    private final Systems systems;

    public ArmTrajectoryCommand(Systems systems, List<PresetPosition> pos, PresetPosition end) {
        this.systems = systems;
        this.setPos = pos;
        this.end = end;
        setName("ArmTrajCommand");
        addRequirements(systems.getArm().getAllComponentsForRequirements());
    }

    @Override
    public void initialize() {
        traj = TrajectoryGenerator.generateTrajectory(systems.getArm().getCurrentPose(), setPos.stream().map((PresetPosition s) -> s.toPose2d().getTranslation()).toList(), end.toPose2d(), Constants.ARM_TRAJECTORY_CONFIG);
        System.out.println(traj.getStates());
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
