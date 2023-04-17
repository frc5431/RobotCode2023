package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Systems;
import frc.robot.util.PresetPosition;

public class ArmTrajectoryCommand extends CommandBase {
    Timer elapsedTime = new Timer();
    private Trajectory traj = null;
    private final PresetPosition setPos;
    private final Systems systems;

    public ArmTrajectoryCommand(Systems systems, PresetPosition pos) {
        this.systems = systems;
        this.setPos = pos;
        setName("ArmTrajCommand");
        addRequirements(systems.getArm().getAllComponentsForRequirements());
    }

    @Override
    public void initialize() {
        List<Translation2d> intermed = new ArrayList<>();
        if (systems.getArm().isGoalBackwards() != setPos.isGoalBackwards())
            intermed.add(Constants.armBackwardsIntermediate); // Intermediate pos
        traj = TrajectoryGenerator.generateTrajectory(systems.getArm().getCurrentPose(), intermed, setPos.toPose2d(), Constants.ARM_TRAJECTORY_CONFIG);
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
        traj = null;
    }

    @Override
    public boolean isFinished() {
        return traj != null && elapsedTime.get() > traj.getTotalTimeSeconds();
    }
}
