package frc.robot.commands;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class RunEndCommand extends FunctionalCommand {
    public RunEndCommand(Runnable onExecute, Consumer<Boolean> onEnd, Subsystem... requirements) {
        super(
            () -> {},
            onExecute,
            onEnd,
            () -> false,
            requirements);
    }
}
