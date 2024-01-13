package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Systems;
import frc.robot.subsystems.ArmContainer;
import frc.robot.util.PresetPosition;

public class ArmToGoalCommand extends Command {

    public static final int USE_PID = 1;
    public static final int FINISH_INSTANTLY = 2;
    public static final int USE_INCHES = 4;

    public static final double DISTANCE_TOLERANCE = Units.inchesToMeters(1.5);

    private final ArmContainer arm;
    private final PIDController xPidController;
    private final PIDController yPidController;
    private final int flags;

    private Translation2d goalPosition;
    private double wristDegrees = Double.NaN;

    public ArmToGoalCommand(Systems systems, PresetPosition presetPosition, int flags) {
        this(systems, presetPosition.getWristPos(), flags);
        this.wristDegrees = presetPosition.getWrist();
        setName(getName()+" wa "+this.wristDegrees);
    }

    public ArmToGoalCommand(Systems systems, Translation2d goalPosition, int flags) {
        this.goalPosition = goalPosition;
        this.arm = systems.getArm();
        this.xPidController = new PIDController(0.05, 0, 0);
        this.yPidController = new PIDController(0.05, 0, 0);
        this.flags = flags;

        if ((flags & USE_INCHES) == USE_INCHES) {
            this.goalPosition = new Translation2d(Units.inchesToMeters(this.goalPosition.getX()), Units.inchesToMeters(this.goalPosition.getY()));
        }

        addRequirements(arm.getAllComponentsForRequirements());
        setName("ArmMoveGoal "+goalPosition.toString());
    }

    private void set(Translation2d pos) {
        arm.setGoal(pos);
        if (!Double.isNaN(wristDegrees)) {
            arm.getWrist().setDegrees(wristDegrees);
        }
    }

    @Override
    public void initialize() {
        xPidController.reset();
        yPidController.reset();

        if((flags & FINISH_INSTANTLY) == FINISH_INSTANTLY) {
            set(goalPosition);
        }
    }

    @Override
    public void execute() {
        var currentPosition = arm.getWristRobotSpacePosition(); // in meters
        var updatedPosition = goalPosition; // in meters, or converted to m from in
        if((flags & USE_PID) == USE_PID && (flags & FINISH_INSTANTLY) == 0) {
            updatedPosition = new Translation2d(
                xPidController.calculate(currentPosition.getX(), goalPosition.getX()),
                yPidController.calculate(currentPosition.getY(), goalPosition.getY())
            );
        }

        set(updatedPosition);
    }

    @Override
    public boolean isFinished() {
        if((flags & FINISH_INSTANTLY) == FINISH_INSTANTLY)
            return true;

        Translation2d pos = arm.getWristRobotSpacePosition();
        double distanceFromGoal = pos.minus(goalPosition).getNorm();
        // boolean wristAtSetpoint = Double.isNaN(wristDegrees) || arm.getWrist().atSetpoint(); // shortcut setpoint check if we didn't even set wrist
        boolean wristAtSetpoint = true;
        return (Math.abs(distanceFromGoal) <= DISTANCE_TOLERANCE) && wristAtSetpoint; // distance should already be absolute, but doesn't hurt to check
    }
}
