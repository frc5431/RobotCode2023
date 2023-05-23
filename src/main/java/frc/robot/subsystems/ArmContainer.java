package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Function;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.PresetPosition;
import frc.team5431.titan.core.misc.Calc;
import frc.team5431.titan.core.misc.KinematicsSolver;
import frc.team5431.titan.core.robot.MotionMagic;

import static edu.wpi.first.math.geometry.Rotation2d.fromDegrees;
import static edu.wpi.first.math.geometry.Rotation2d.fromRadians;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.math.util.Units.radiansToDegrees;

public class ArmContainer {
    private final ArmComponent outerComponent;
    private final ArmComponent innerComponent;
    private final ArmComponent wristComponent;

    @SuppressWarnings("unused")
    private final Manipulator manipulator;

    public static final double MAX_SPEED_OUTER = 0.8; // 0.073 to hold at horz
    public static final double MAX_SPEED_INNER = 1.0;  // 0.18 to hold at horz
    public static final double MAX_SPEED_WRIST = 0.5;  // 0.064 to hold at horz

    private static final Rotation2d DEG_90 = fromDegrees(90);
    private static final double TORQUE_NM_NEO = 2.6;

    public static final double SHOULDER_TORQUE_TOTAL = TORQUE_NM_NEO * 100 * 2;
    public static final double FOREARM_TORQUE_TOTAL = TORQUE_NM_NEO * 5*4 * 84/20 * 2;
    public static final double WRIST_TORQUE_TOTAL = TORQUE_NM_NEO * 5*3;

    private final double SETPOINT_POSITION_TOLERANCE = 0.1;
    private final double SETPOINT_VELOCITY_TOLERANCE = 0.2;

    public static final KinematicsSolver solver = new KinematicsSolver(Units.inchesToMeters(24), Units.inchesToMeters(20)); // 34, 26

    private Translation2d goalPose = new Translation2d(Units.inchesToMeters(50), -Units.inchesToMeters(30)); // x = 5
    public static final double IS_BACKWARDS_X = Units.inchesToMeters(-16);

    // elec is 0.186 kg
    public static final double shoulderMassKG = 10.02; // 10.259 measured w/elec // 3.1+1.877+5.096 w/o elec
    public static final double elbowMassKG = 4.347; // w/elec
    public static final double wristMassKG = 2.42; // 3.1 // w/elec
    public static final double coneMassKG = 0.67; // 0.8 // 0.67 // 0.9
    public static final double GRAV_CONST = 9.81;

    // ((mass (kg) * acceleration (m/s/s)) (N) * distance of center of mass from pivot (m)) (Nm)
    public static final double shoulderCosineMultiplierNoCOM =
        shoulderMassKG * GRAV_CONST;

    public static final double shoulderMinCOMMeters =
        Units.inchesToMeters(13.029); // 0.473 meters

    public static final double shoulderMaxCOMMeters =
        Units.inchesToMeters(32); // 26.530

    public static final double elbowCosineMultiplierNoCOM =
        elbowMassKG * GRAV_CONST;

    public static final double elbowMinCOMMeters =
        Units.inchesToMeters(13.127); // 20

    public static final double elbowMaxCOMMeters =
        Units.inchesToMeters(17.673);

    public static final double wristCosineMultiplierNoCOM = 
        wristMassKG * GRAV_CONST;

    public static final double wristCOMMeters =
        Units.inchesToMeters(3); // 3.5

    private final List<CANSparkMax> sparks;

    private PresetPosition intermediateHighPosition = Constants.armHighIntermediate; // default to existing high intermed

    /* Arm encoder directions (robot facing right)
     * - shoulder
     *   - encoder CCW+
     *   - 0 facing down
     * - elbow
     *   - motor CW+
     *   - encoder CW+
     *   - 0 facing away from bicep
     * - wrist
     *   - motor CCW+
     *   - encoder CCW+
     *   - 0 facing away from forearm
     */

    public ArmContainer(Manipulator manip, CANSparkMax outerArmLeft, CANSparkMax outerArmRight, CANSparkMax innerArmLeft, CANSparkMax innerArmRight, CANSparkMax wrist) {
        this.manipulator = manip;
        outerArmLeft.setInverted(false);
        outerArmRight.follow(outerArmLeft, true);
        outerArmLeft.setIdleMode(IdleMode.kBrake);
        outerArmRight.setIdleMode(IdleMode.kBrake);

        outerArmLeft.enableSoftLimit(SoftLimitDirection.kForward, false);
        outerArmLeft.enableSoftLimit(SoftLimitDirection.kReverse, false);
        // outerArmLeft.setSoftLimit(SoftLimitDirection.kForward, 0.80f);
        // outerArmLeft.setSoftLimit(SoftLimitDirection.kReverse, 0.20f);

        innerArmRight.setInverted(false);
        innerArmLeft.follow(innerArmRight, true);
        innerArmRight.setIdleMode(IdleMode.kBrake);
        innerArmLeft.setIdleMode(IdleMode.kBrake);

        innerArmRight.enableSoftLimit(SoftLimitDirection.kForward, false);
        innerArmRight.enableSoftLimit(SoftLimitDirection.kReverse, false);
        // innerArmRight.setSoftLimit(SoftLimitDirection.kForward, 0.80f);
        // innerArmRight.setSoftLimit(SoftLimitDirection.kReverse, 0.20f);

        wrist.setInverted(false);
        wrist.setIdleMode(IdleMode.kBrake);

        wrist.enableSoftLimit(SoftLimitDirection.kForward, false);
        wrist.enableSoftLimit(SoftLimitDirection.kReverse, false);
        // wrist.setSoftLimit(SoftLimitDirection.kForward, (float) ( Math.PI/2 + 0.1));
        // wrist.setSoftLimit(SoftLimitDirection.kReverse, (float) ((-Math.PI/2 - 0.1)));

        sparks = List.of(outerArmLeft, outerArmRight, innerArmLeft, innerArmRight, wrist);

        sparks.forEach((spark) -> {
            // spark.enableVoltageCompensation(12.0);
            spark.disableVoltageCompensation();
            spark.setSmartCurrentLimit(60, 35); // 40
            spark.burnFlash();
        });

        outerComponent = new ArmComponent("shoulder", "bicep", outerArmLeft, outerArmRight, new MotionMagic(1.0, 0.0, 0.0, 0.0), MAX_SPEED_OUTER, (component) -> {
            Rotation2d ba2g = component.angle2Ground.apply(fromRadians(component.getSetpointRadians()));
            double arbFF = getShoulderCosMult() * ba2g.getCos() / SHOULDER_TORQUE_TOTAL;

            component.getController().setReference(component.getSetpointRadians(), ControlType.kPosition, 0, arbFF, ArbFFUnits.kPercentOut);
            SmartDashboard.putNumber("shoulder set", component.getSetpointRadians());
            SmartDashboard.putNumber("shoulder arbff", arbFF);
        }, (bicepAngle) -> calcBicepAngleToGround(bicepAngle), Pair.of(-Math.PI, Math.PI));

        innerComponent = new ArmComponent("elbow", "forearm", innerArmRight, innerArmLeft, new MotionMagic(1.5, 0.0, 0.01, 0.0), MAX_SPEED_INNER, (component) -> {
            Rotation2d fa2g = component.angle2Ground.apply(fromRadians(component.getSetpointRadians()));
            double arbFF = -getElbowCosMult() * fa2g.getCos() / FOREARM_TORQUE_TOTAL;

            component.getController().setReference(component.getSetpointRadians(), ControlType.kPosition, 0, arbFF, ArbFFUnits.kPercentOut);
            SmartDashboard.putNumber("elbow set", component.getSetpointRadians());
            SmartDashboard.putNumber("elbow arbff", arbFF);
        }, (forearmAngle) -> calcForearmAngleToGround(outerComponent.getPositionRot2d(), forearmAngle), Pair.of(-Math.PI+0.3, Math.PI-0.3));

        wristComponent = new ArmComponent("wrist", "hand", wrist, new MotionMagic(0.4, 0.0, 0.0, 0.0), MAX_SPEED_WRIST, (component) -> {
            Rotation2d wa2g = component.angle2Ground.apply(fromRadians(component.getSetpointRadians()));
            double arbFF = getWristCosMult() * wa2g.getCos() / WRIST_TORQUE_TOTAL;

            component.getController().setReference(component.getSetpointRadians(), ControlType.kPosition, 0, arbFF, ArbFFUnits.kPercentOut);
            SmartDashboard.putNumber("wa2g", wa2g.getDegrees());
            SmartDashboard.putNumber("wrist set", component.getSetpointRadians());
            SmartDashboard.putNumber("wrist setdeg", component.getSetpointDegrees());
            SmartDashboard.putNumber("wrist arbff", arbFF);
        }, (handAngle) -> calcHandAngleToGround(outerComponent.getPositionRot2d(), innerComponent.getPositionRot2d(), handAngle) , Pair.of(3.92-2*Math.PI, 0.8));
    }

    public void setIntermediatePosition() {
        intermediateHighPosition = PresetPosition.fromGoal(goalPose, wristComponent.getPositionDegrees(), false).metersToInches();
    }

    public PresetPosition getIntermediatePostion() {
        return intermediateHighPosition;
    }

    public ArmComponent getOuter() {
        return outerComponent;
    }

    public ArmComponent getInner() {
        return innerComponent;
    }

    public ArmComponent getWrist() {
        return wristComponent;
    }

    public ArmComponent[] getAllComponentsForRequirements() {
        return new ArmComponent[]{ outerComponent, innerComponent, wristComponent };
    }

    public void setGoal(Translation2d v) {
        goalPose = v;
        solveKinematics(goalPose);
    }

    public Translation2d getGoal() {
        return goalPose;
    }

    public Rotation2d calcBicepAngleToGround(Rotation2d bicepAngle) {
        return bicepAngle.minus(DEG_90);
    }

    public Rotation2d calcForearmAngleToGround(Rotation2d bicepAngle, Rotation2d forearmAngle) {
        return calcBicepAngleToGround(bicepAngle).minus(forearmAngle);
    }

    public Rotation2d calcHandAngleToGround(Rotation2d bicepAngle, Rotation2d forearmAngle, Rotation2d handAngle) {
        return calcForearmAngleToGround(bicepAngle, forearmAngle).plus(handAngle);
    }

    public double getShoulderCosMult() {
        return shoulderCosineMultiplierNoCOM * getCOMBicepMeters();
        // if (manipulator.getHeldGamePiece() != Manipulator.GamePiece.CONE) {
        //     return shoulderCosineMultiplierNoCOM * getCOMBicepMeters();
        // } else {
        //     double coneDistance = this.goalPose.getNorm() + Units.inchesToMeters(7);
        //     double totalMass = shoulderMassKG + coneMassKG;
        //     double newCOM = (shoulderMassKG * getCOMBicepMeters() + coneMassKG * coneDistance) / totalMass;
        //     SmartDashboard.putNumber("shcom wo cone", shoulderMassKG * getCOMBicepMeters());
        //     SmartDashboard.putNumber("shcom wt cone", totalMass * newCOM);
        //     return totalMass * newCOM * GRAV_CONST;
        // }
    }

    public double getElbowCosMult() {
        return elbowCosineMultiplierNoCOM * getCOMForearmMeters();
        // if (manipulator.getHeldGamePiece() != Manipulator.GamePiece.CONE) {
        //     return elbowCosineMultiplierNoCOM * getCOMForearmMeters();
        // } else {
        //     double coneDistance = Units.inchesToMeters(20+7);
        //     double totalMass = elbowMassKG + coneMassKG;
        //     double newCOM = (elbowMassKG * getCOMForearmMeters() + coneMassKG * coneDistance) / totalMass;
        //     SmartDashboard.putNumber("elcom wo cone", elbowMassKG * getCOMForearmMeters());
        //     SmartDashboard.putNumber("elcom wt cone", totalMass * newCOM);
        //     return totalMass * newCOM * GRAV_CONST;
        // }
    }

    public double getWristCosMult() {
        return wristCosineMultiplierNoCOM * wristCOMMeters;
        // if (manipulator.getHeldGamePiece() != Manipulator.GamePiece.CONE) {
        //     return wristCosineMultiplierNoCOM * wristCOMMeters;
        // } else {
        //     double coneDistance = Units.inchesToMeters(7); // 12
        //     double totalMass = wristMassKG + coneMassKG;
        //     double newCOM = (wristMassKG * wristCOMMeters + coneMassKG * coneDistance) / totalMass;
        //     SmartDashboard.putNumber("wrcom wo cone", wristMassKG * wristCOMMeters);
        //     SmartDashboard.putNumber("wrcom wt cone", totalMass * newCOM);
        //     return totalMass * newCOM * GRAV_CONST;
        // }
    }

    public double getCOMBicepMeters() {
        double mapped = -innerComponent.getPositionRadians();
        return Calc.map(Math.cos(mapped), 1, -1, shoulderMaxCOMMeters, shoulderMinCOMMeters);
    }

    public double getCOMForearmMeters() {
        double mapped = wristComponent.getPositionRadians();
        return Calc.map(Math.cos(mapped), 1, 0, elbowMaxCOMMeters, elbowMinCOMMeters);
    }

    public List<CANSparkMax> getSparks() {
        return sparks;
    }

    public Command defaultCommand(DoubleSupplier outSupplier, DoubleSupplier innerSupplier, DoubleSupplier wristSupplier) {
        return Commands.runEnd(
            () -> {
                this.getOuter().setSpeed(outSupplier.getAsDouble());
                this.getInner().setSpeed(innerSupplier.getAsDouble());
                this.getWrist().setSpeed(wristSupplier.getAsDouble());
            },
            () -> {
                this.getOuter().setSpeed(0);
                this.getInner().setSpeed(0);
                this.getWrist().setSpeed(0);
            },
            getOuter(), getInner(), getWrist());
    }

    public Command defaultCommand(DoubleSupplier outSupplier, DoubleSupplier innerSupplier) {
        return Commands.runEnd(
            () -> {
                this.getOuter().setSpeed(outSupplier.getAsDouble());
                this.getInner().setSpeed(innerSupplier.getAsDouble());
            },
            () -> {
                this.getOuter().setSpeed(0);
                this.getInner().setSpeed(0);
            },
            getOuter(), getInner(), getWrist());
    }

    public void setGoalToCurrentPosition() {
        Translation2d newGoal = solver.anglesToPos(outerComponent.getPositionRadians(), innerComponent.getPositionRadians());
        setGoal(newGoal);
    }

    public void solveKinematics(Translation2d goal) {
        Pair<Double, Double> ik = solver.posToAngles(goal, solver.useTopByDefault);
        if (!Double.isNaN(ik.getFirst()))
            getOuter().setRadians(ik.getFirst());
        if (!Double.isNaN(ik.getSecond()))
            getInner().setRadians(ik.getSecond());

        SmartDashboard.putNumber("Goal X", Units.metersToInches(goal.getX()));
        SmartDashboard.putNumber("Goal Y", Units.metersToInches(goal.getY()));
        SmartDashboard.putNumber("InvKin Out", ik.getFirst());
        SmartDashboard.putNumber("InvKin In",  ik.getSecond());
    }

    public Translation2d getWristRobotSpacePosition() {
        return solver.anglesToPos(outerComponent.getPositionRadians(), innerComponent.getPositionRadians());
    }

    public Pose2d getCurrentPose() {
        return new Pose2d(getWristRobotSpacePosition(), wristComponent.getPositionRot2d());
    }

    public boolean isGoalBackwards() {
        return isPoseBackwards(goalPose);
    }

    public boolean isPoseBackwards(Translation2d pose) {
        return pose.getX() < IS_BACKWARDS_X;
    }

    Translation2d old_pos = null;
    double old_speed = 0;
    double max_speed;
    double max_acceleration;
    public void debugPeriodic() {
        if(old_pos == null) {
            old_pos = getWristRobotSpacePosition();
        }
        double speed =  getWristRobotSpacePosition().getDistance(old_pos) / 0.02;
        if(max_speed < speed) {
            max_speed = speed;
        }
        if(max_acceleration < (speed - old_speed) / 0.02) {
            max_acceleration = (speed - old_speed) / 0.02;
        }
        SmartDashboard.putNumber("arm spd", speed);
        SmartDashboard.putNumber("arm accel", (speed - old_speed) / 0.02);
        SmartDashboard.putNumber("max arm spd", max_speed);
        SmartDashboard.putNumber("max arm accel", max_acceleration);
        old_pos = getWristRobotSpacePosition();
        old_speed = speed;

        SmartDashboard.putString("high intermed", intermediateHighPosition.toString());
    }

    public class ArmComponent extends SubsystemBase {
        private final CANSparkMax motor;
        private final Optional<CANSparkMax> follow;
        private final SparkMaxPIDController controller;
        private final AbsoluteEncoder absoluteEncoder;
        public final Consumer<ArmComponent> setter;
        public final Function<Rotation2d, Rotation2d> angle2Ground;
        public final double MAX_SPEED;
        public final String jointName;
        public final String segmentName;

        private final Pair<Double, Double> setpointClamp;

        private double setpoint;

        public ArmComponent(String jointName, String segmentName, CANSparkMax motor, CANSparkMax follow, MotionMagic pidConstants, double maxSpeed, Consumer<ArmComponent> setter, Function<Rotation2d, Rotation2d> angle2Ground, Pair<Double, Double> clamp) {
            this.jointName = jointName;
            this.segmentName = segmentName;
            this.motor = motor;
            this.follow = Optional.ofNullable(follow);

            controller = motor.getPIDController();
            absoluteEncoder = motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
            this.setter = setter;
            this.angle2Ground = angle2Ground;
            this.MAX_SPEED = maxSpeed;
            this.setpointClamp = clamp;

            controller.setP(pidConstants.p());
            controller.setI(pidConstants.i());
            controller.setD(pidConstants.d());
            controller.setFF(pidConstants.f());
            controller.setOutputRange(-MAX_SPEED, MAX_SPEED);
            controller.setPositionPIDWrappingEnabled(true);
            controller.setPositionPIDWrappingMinInput(0);
            controller.setPositionPIDWrappingMaxInput(2*Math.PI);

            absoluteEncoder.setPositionConversionFactor(2*Math.PI);
            absoluteEncoder.setVelocityConversionFactor(2*Math.PI);

            controller.setFeedbackDevice(absoluteEncoder);

            motor.burnFlash();
        }

        public ArmComponent(String jointName, String segmentName, CANSparkMax motor, MotionMagic pidConstants, double maxSpeed, Consumer<ArmComponent> setter, Function<Rotation2d, Rotation2d> angle2ground, Pair<Double, Double> clamp) {
            this(jointName, segmentName, motor, null, pidConstants, maxSpeed, setter, angle2ground, clamp);
        }

        double maxSpeed = 0;
        @Override
        public void periodic() {
            final Rotation2d angle = getPositionRot2d();

            SmartDashboard.putNumber(jointName + " spd", motor.getAppliedOutput());
            SmartDashboard.putNumber(jointName + " cur", angle.getRadians());
            SmartDashboard.putNumber(segmentName + " angle", angle.getDegrees());
            SmartDashboard.putNumber(segmentName + " angle to ground", angle2Ground.apply(angle).getDegrees());
            SmartDashboard.putBoolean(jointName + " atSetpoint", atSetpoint());
            SmartDashboard.putNumber(jointName + " error", 
                Math.abs(fromRadians(setpoint).minus(getPositionRot2d()).getRadians()));
            if (follow.isPresent()) {
                SmartDashboard.putNumber((jointName.charAt(0)+"L").toUpperCase()+" Amps", motor.getOutputCurrent());
                SmartDashboard.putNumber((jointName.charAt(0)+"F").toUpperCase()+" Amps", follow.get().getOutputCurrent());
            } else SmartDashboard.putNumber(jointName + " amps", motor.getOutputCurrent());
            if(motor.getAppliedOutput() > maxSpeed) {
                maxSpeed = motor.getAppliedOutput();
            }
            SmartDashboard.putNumber(jointName + " max spd", maxSpeed);
            runSetter();
        }


        public double getSetpointRadians() {
            return setpoint;
        }

        public double getSetpointDegrees() {
            return radiansToDegrees(getSetpointRadians());
        }

        public double getPositionRadians() {
            return absoluteEncoder.getPosition();
        }

        public double getPositionDegrees() {
            return radiansToDegrees(getPositionRadians());
        }

        public Rotation2d getPositionRot2d() {
            return fromRadians(getPositionRadians());
        }


        public void setDegrees(double value) {
            setRadians(degreesToRadians(value));
        }

        public void setRadians(double value) {
            setpoint = MathUtil.clamp(MathUtil.inputModulus(value, -Math.PI, Math.PI), setpointClamp.getFirst(), setpointClamp.getSecond());
            runSetter();
        }

        public void add(double degrees) {
            setDegrees(getSetpointDegrees() + degrees);
        }

        public void setSpeed(double value) {
            double modSpd = MathUtil.clamp(value, -1, 1);
            motor.set(modSpd * MAX_SPEED);
        }

        public boolean atSetpoint() {
            Rotation2d currentPosition = fromRadians(absoluteEncoder.getPosition());
            Rotation2d setpointRot2d = fromRadians(setpoint);
            return Math.abs(setpointRot2d.minus(currentPosition).getRadians()) < SETPOINT_POSITION_TOLERANCE 
                && Math.abs(absoluteEncoder.getVelocity()) < SETPOINT_VELOCITY_TOLERANCE
                ;
        }

        public CANSparkMax getMotor() {
            return motor;
        }

        public Optional<CANSparkMax> getFollow() {
            return follow;
        }

        public SparkMaxPIDController getController() {
            return controller;
        }

        public AbsoluteEncoder getEncoder() {
            return absoluteEncoder;
        }

        public void runSetter() {
            setter.accept(this);
        }

        // Will currently require all of the arm, so no other command can
        // be scheduled in parallel
        // TODO make ArmComponent into its own SubsystemBase
        public Command setDegreesCommand(double degrees) {
            return runOnce(() -> this.setDegrees(degrees));
                    // .andThen(Commands.waitUntil(this::atSetpoint));
        }

        public Command setRadiansCommand(double radians) {
            return runOnce(() -> this.setRadians(radians));
                    // .andThen(Commands.waitUntil(this::atSetpoint));
        }
    }
}
