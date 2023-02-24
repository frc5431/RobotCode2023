package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Manipulator;

public class WristOpenCommand extends CommandBase {
    Manipulator manip;
    boolean open;
    public WristOpenCommand(Manipulator manip, boolean open) {
        this.manip = manip;
        this.open = open;
    }

    @Override
    public void initialize() {
        manip.setState(manip.getStateFromBool(open));
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
