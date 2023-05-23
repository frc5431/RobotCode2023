package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import java.util.stream.Stream;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.Constants;
import frc.robot.Systems;
import frc.robot.util.PresetPosition;

public final class ArmTrajectoryCommandFactory {
    private ArmTrajectoryCommandFactory() {}


    public static Command procure(Systems systems, PresetPosition... goalPositions) {
        return procure(systems, Stream.of(goalPositions).toList());
    }

    public static Command procure(Systems systems, TrajectoryConfig config, PresetPosition... goalPositions) {
        return procure(systems, Stream.of(goalPositions).toList(), config);
    }

    @SafeVarargs
    public static Command procure(Systems systems, TrajectoryConfig config, Supplier<PresetPosition>... goalPositions) {
        return new ProxyCommand(() -> {
            List<PresetPosition> poses = new ArrayList<>();
            for (var poseSupp : goalPositions) {
                try {
                    poses.add(poseSupp.get());
                } catch(Exception ignored) {
                    System.out.println("test action attempted before supplier result could be resolved.");
                }
            }
            return procure(systems, poses, config);
        });
    }

    public static Command procure(Systems systems, List<PresetPosition> goalPositions) {
        return procure(systems, goalPositions, Constants.ARM_TRAJECTORY_CONFIG);
    }

    /**
     * 
     * @param systems
     * @param goalPositions A list of goal positions in inches
     * @return a command that takes you from current position to end position going through all goal positions
     */
    public static Command procure(Systems systems, List<PresetPosition> goalPositions, TrajectoryConfig config) {
        return new ProxyCommand(() -> {
            System.out.println(goalPositions);

            if(goalPositions.size() == 1 && goalPositions.get(0).inchesToMeters().isGoalBackwards() == systems.getArm().isGoalBackwards()) {
                return new ArmToGoalCommand(systems, goalPositions.get(0), ArmToGoalCommand.USE_INCHES | ArmToGoalCommand.FINISH_INSTANTLY);
            }
            List<PresetPosition> positions = new ArrayList<>();
            for(int i = 0; i < goalPositions.size(); i++) {
                var pos = goalPositions.get(i);

                // Incorperate Intermediate position if neccisary
                PresetPosition prevPos;
                if((i - 1) < 0) {
                    prevPos = PresetPosition.fromGoal(systems.getArm().getCurrentPose().getTranslation(), systems.getArm().getWrist().getPositionDegrees(), false);
                }else {
                    prevPos = positions.get(i -1);
                }
                System.out.println(pos.isGoalBackwards() != prevPos.isGoalBackwards());
                if(pos.isGoalBackwards() != prevPos.isGoalBackwards()) {
                    positions.add(Constants.armToBackIntermediate.inchesToMeters());
                }
                if(i != goalPositions.size() - 1)
                    positions.add(pos.inchesToMeters());
            }
            System.out.println(positions);
            return new ArmTrajectoryCommand(systems, positions, goalPositions.get(goalPositions.size() - 1).inchesToMeters(), config);
        });
    }
}
