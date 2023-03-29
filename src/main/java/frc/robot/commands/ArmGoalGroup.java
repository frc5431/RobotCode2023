package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Systems;
import frc.robot.util.PresetPosition;

public class ArmGoalGroup extends SequentialCommandGroup {
    public ArmGoalGroup(Systems systems, PresetPosition presetPosition, int flags) {
        addCommands(
            // either(new ArmToGoalCommand(
            //     systems,
            //     Constants.armBackwardsIntermediate,
            //     ArmToGoalCommand.USE_INCHES
            // ), none(), () -> (systems.getArm().isGoalBackwards() != presetPosition.isGoalBackwards())),
            new ArmToGoalCommand(systems, presetPosition, flags)
        );
    }

    public ArmGoalGroup(Systems systems, Translation2d goalPosition, int flags) {
        addCommands(
            // either(new ArmToGoalCommand(
            //     systems,
            //     Constants.armBackwardsIntermediate,
            //     ArmToGoalCommand.USE_INCHES
            // ), none(), () -> (systems.getArm().isGoalBackwards() != PresetPosition.isGoalBackwards(goalPosition))),
            new ArmToGoalCommand(systems, goalPosition, flags)
        );
    }
}
