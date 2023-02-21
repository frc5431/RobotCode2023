package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Systems;

public class ArmMoveCommandGroup extends ParallelCommandGroup {
    public ArmMoveCommandGroup(Systems systems, Translation2d position, int jumpFlags, double wristAngle, boolean open) {
        addCommands(
            new ArmToGoalCommand(systems, position, jumpFlags),
            new WristAngleCommand(systems.getArm().getWrist(), wristAngle),
            new WristOpenCommand(systems.getManipulator(), open)
        );
    }
}
