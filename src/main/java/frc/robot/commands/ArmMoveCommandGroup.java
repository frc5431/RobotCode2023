package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Systems;
import frc.robot.util.PresetPosition;

public class ArmMoveCommandGroup extends ParallelCommandGroup {
    public ArmMoveCommandGroup(Systems systems, Translation2d position, int jumpFlags, double wristAngle, boolean open) {
        addCommands(
            new ArmToGoalCommand(systems, PresetPosition.fromGoal(position, wristAngle), jumpFlags),
            systems.getManipulator().manipCommand(open)
        );
    }
}
