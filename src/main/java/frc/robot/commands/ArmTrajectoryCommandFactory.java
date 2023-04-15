package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Systems;
import frc.robot.util.PresetPosition;

public final class ArmTrajectoryCommandFactory {
    private ArmTrajectoryCommandFactory() {}


    public static Command procure(Systems systems, PresetPosition... goalPositions) {
        return procure(systems, Stream.of(goalPositions).toList());
    }

    /**
     * 
     * @param systems
     * @param goalPositions A list of goal positions in inches
     * @return a command that takes you from current position to end position going through all goal positions
     */
    public static Command procure(Systems systems, List<PresetPosition> goalPositions) {
        if(goalPositions.size() == 1 && goalPositions.get(0).isGoalBackwards() != systems.getArm().isGoalBackwards()) {
            return new ArmToGoalCommand(systems, goalPositions.get(0), ArmToGoalCommand.USE_INCHES);
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

            if(pos.isGoalBackwards() != prevPos.isGoalBackwards()) {
                positions.add(Constants.armToBackIntermediary);
            }
            if(i != goalPositions.size() - 1)
                positions.add(pos);
        }
        return new ArmTrajectoryCommand(systems, positions, goalPositions.get(goalPositions.size() - 1));
    }


}
