package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Manipulator;

public class ArmMoveCommandGroup extends ParallelCommandGroup {
    public ArmMoveCommandGroup(Arm arm, Manipulator manip, Translation2d position, int jumpFlags, double wristAngle, boolean open) {
        addCommands(
            new JumpToGoalPositionCommand(arm, position, jumpFlags),
            new WristAngleCommand(arm.getWrist(), wristAngle),
            new WristOpenCommand(manip, open)
        );
    }
}
