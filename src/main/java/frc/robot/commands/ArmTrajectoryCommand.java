package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Systems;
import frc.robot.util.PresetPosition;

public class ArmTrajectoryCommand extends Command {
    Timer elapsedTime = new Timer();
    private Trajectory traj = null;
    private final List<PresetPosition> setPos;
    private final PresetPosition end;
    private final Systems systems;
    private final TrajectoryConfig config;

    public ArmTrajectoryCommand(Systems systems, List<PresetPosition> pos, PresetPosition end) {
        this(systems, pos, end, Constants.ARM_TRAJECTORY_CONFIG);
    }

    public ArmTrajectoryCommand(Systems systems, List<PresetPosition> pos, PresetPosition end, TrajectoryConfig config) {
        this.systems = systems;
        this.setPos = pos;
        this.end = end;
        this.config = config;
        setName("ArmTrajCommand");
        
        //addRequirements(systems.getArm().getAllComponentsForRequirements());
    }

    @Override
    public void initialize() {
        traj = TrajectoryGenerator.generateTrajectory(
            new Pose2d(systems.getArm().getGoal(), new Rotation2d()),
            setPos.stream().map((PresetPosition s) -> s.toPose2d().getTranslation()).toList(),
            end.toPose2d(),
            config);
        System.out.println("Starting arm traj with time " + traj.getTotalTimeSeconds());
        elapsedTime.restart();
    }

    @Override
    public void execute() {
        var state = traj.sample(elapsedTime.get());
        Pose2d set = state.poseMeters;
        systems.getArm().setGoal(set.getTranslation());
        if(set.getTranslation().getDistance(setPos.get(0).getWristPos()) < 0.1) {
            System.out.println("Approached within 10cm of setpoint 1");
        }
        // System.out.println("set: " + set);
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
