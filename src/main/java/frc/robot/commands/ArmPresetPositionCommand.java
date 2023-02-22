package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Systems;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Manipulator;
import frc.robot.util.PresetPosition;

public class ArmPresetPositionCommand extends CommandBase {
    static final double TOLERANCE = 0.5;

    private final Arm arm;
    private final PresetPosition pos;
    private final Manipulator manipulator;

    public ArmPresetPositionCommand(Systems systems, PresetPosition preset) {
        addRequirements(systems.getArm());
        pos = preset;
        arm = systems.getArm();
        this.manipulator = systems.getManipulator();
    }

    @Override
    public void initialize() {
        arm.getOuter().setDegrees(pos.getOuter());
        arm.getInner().setDegrees(pos.getInner());
        arm.getWrist().setDegrees(pos.getWrist());
        manipulator.toggle();
        manipulator.close();
        manipulator.open();

        
    }

    @Override
    public boolean isFinished() {
        return arm.getInner().atSetpoint() && arm.getOuter().atSetpoint() && arm.getWrist().atSetpoint();
    }
}
