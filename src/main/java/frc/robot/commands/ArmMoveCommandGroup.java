package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Systems;
import frc.robot.util.PresetPosition;

public class ArmMoveCommandGroup extends SequentialCommandGroup {
    public ArmMoveCommandGroup(Systems systems, Translation2d position, int jumpFlags, double wristAngle, boolean open) {
        addCommands(
            systems.getManipulator().manipCommand(open),
            new ArmToGoalCommand(systems, PresetPosition.fromGoal(position, wristAngle), jumpFlags)
        );
        setName("ArmMoveGroup to "+position.toString()+" wa "+wristAngle);
    }
}
