package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Systems;
import frc.robot.subsystems.Arm;
import frc.robot.util.PresetPosition;

public class ArmPresetPositionCommand extends CommandBase {
    static final double TOLERANCE = 0.5;

    private final Arm arm;
    private final PresetPosition pos;
    public ArmPresetPositionCommand(Systems systems, double outer, double inner, double wrist) {
        this(systems, new PresetPosition(outer, inner, wrist));
    }

    public ArmPresetPositionCommand(Systems systems, PresetPosition preset) {
        addRequirements(systems.getArm());
        pos = preset;
        arm = systems.getArm();
    }

    @Override
    public void initialize() {
        arm.setOut(pos.getOuter());
        arm.setIn(pos.getInner());
        arm.setWrist(pos.getWrist());
    }

    @Override
    public boolean isFinished() {
        return arm.elbowAtSetpoint(pos.getOuter()) && arm.shoulderAtSetpoint(pos.getInner()) && arm.wristAtSetpoint(pos.getWrist());
    }
}
