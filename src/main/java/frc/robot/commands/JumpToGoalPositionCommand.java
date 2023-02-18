package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class JumpToGoalPositionCommand extends CommandBase {
    public static final int USE_PID = 1;
    public static final int FINISH_INSTANTLY = 2;

    public static final double distanceTolerance = Units.inchesToMeters(1.5);
    
    private int flags = 0;
    private Translation2d goalPosition;
    private Arm arm;
    private PIDController xPidController = new PIDController(0.05, 0, 0);
    private PIDController yPidController = new PIDController(0.05, 0, 0);


    public JumpToGoalPositionCommand(Arm arm, Translation2d goalPosition) {
        this(arm, goalPosition, USE_PID);
    }

    public JumpToGoalPositionCommand(Arm arm, Translation2d goalPosition, int flags) {
        this.goalPosition = goalPosition;
        this.arm = arm;
        this.flags = flags;
    }

    @Override
    public void initialize() {
        xPidController.reset();
        yPidController.reset();
    }

    @Override
    public void execute() {
        var currentPosition = arm.getWristRobotSpacePosition();
        var updatedPosition = goalPosition;
        if((flags & USE_PID) == USE_PID && (flags & FINISH_INSTANTLY) == 0) {
            updatedPosition = new Translation2d(xPidController.calculate(currentPosition.getX(), goalPosition.getX()), yPidController.calculate(currentPosition.getY(), goalPosition.getY()));
        }
        arm.setGoal(updatedPosition);
    }

    @Override
    public boolean isFinished() {
        if((flags & FINISH_INSTANTLY) == FINISH_INSTANTLY) {
            return true;
        }
        Translation2d pos = arm.getWristRobotSpacePosition();
        Translation2d dif = new Translation2d(Math.abs(pos.getX()  - goalPosition.getX()), Math.abs(pos.getY() - goalPosition.getY()));
        if(dif.getX() < distanceTolerance && dif.getY() < distanceTolerance) {
            return true;
        }
        return false;
    }
}
