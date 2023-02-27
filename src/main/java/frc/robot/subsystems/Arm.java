package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm.ArmComponent.ComponentCalculationConstants;
import frc.robot.util.KinematicsSolver;
import frc.team5431.titan.core.misc.Calc;
import frc.team5431.titan.core.robot.MotionMagic;

import static edu.wpi.first.math.geometry.Rotation2d.fromDegrees;
import static edu.wpi.first.math.geometry.Rotation2d.fromRadians;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.math.util.Units.radiansToDegrees;

public class Arm extends SubsystemBase {
    private final ArmComponent outerComponent;
    private final ArmComponent innerComponent;
    private final ArmComponent wristComponent;

    public static final double MAX_SPEED_OUTER = 0.2; // 0.073 to hold at horz
    public static final double MAX_SPEED_INNER = 0.21;  // 0.18 to hold at horz
    public static final double MAX_SPEED_WRIST = 0.2;  // 0.064 to hold at horz

    // private double setpointOut = 0;
    // private double setpointIn = 0;
    // private double setpointWrist = 0;

    private static final Rotation2d DEG_90 = fromDegrees(90);
    private static final double TORQUE_NM_NEO = 2.6;

    public static final double SHOULDER_TORQUE_TOTAL = TORQUE_NM_NEO * 100 * 2;
    public static final double ELBOW_TORQUE_TOTAL = TORQUE_NM_NEO * 5*4 * 84/20 * 2;
    public static final double WRIST_TORQUE_TOTAL = TORQUE_NM_NEO * 3*3;

    // measured
    private Rotation2d bicepAngle = new Rotation2d();
    private Rotation2d bicepAngleToGround = new Rotation2d();
    private Rotation2d forearmAngle = new Rotation2d();
    private Rotation2d forearmAngleToGround = new Rotation2d();
    private Rotation2d handAngle = new Rotation2d();
    private Rotation2d handAngleToGround = new Rotation2d();

    private final double SETPOINT_POSITION_TOLERANCE = 0.1;
    private final double SETPOINT_VELOCITY_TOLERANCE = 0.2;

    public static final KinematicsSolver solver = new KinematicsSolver(Units.inchesToMeters(34), Units.inchesToMeters(26));

    private Translation2d goalPose = new Translation2d(Units.inchesToMeters(50), -Units.inchesToMeters(30)); // x = 5

    public static final double shoulderMassKG = 8.15;
    public static final double elbowMassKG = 3.0;
    public static final double wristMassKG = 1.85;
    public static final double coneMassKG = 0.66;
    public static final double GRAV_CONST = 9.81;
    public static final double metersFromWristToGamePiece = Units.inchesToMeters(11);

    // ((mass (kg) * acceleration (m/s/s)) (N) * distance of center of mass from pivot (m)) (Nm)
    public static final double shoulderCosineMultiplierNoCOM =
        shoulderMassKG * GRAV_CONST;

    public static final double shoulderMinCOMMeters =
        Units.inchesToMeters(18.624); // 0.473 meters

    public static final double shoulderMaxCOMMeters =
        Units.inchesToMeters(40.625);

    public static final double elbowCosineMultiplierNoCOM =
        elbowMassKG * GRAV_CONST;

    public static final double elbowMinCOMMeters =
        Units.inchesToMeters(17.07);

    public static final double elbowMaxCOMMeters =
        Units.inchesToMeters(22.93);

    public static final double wristCosineMultiplierNoCOM = 
        wristMassKG * GRAV_CONST;

    public static final double wristCOMMeters =
        Units.inchesToMeters(3.1);

    public double getShoulderCosMult() {
        if (Manipulator.isOpen) {
            return shoulderCosineMultiplierNoCOM * getCOMBicepMeters();
        } else {
            double coneDistance = this.goalPose.getNorm() + Units.inchesToMeters(11);
            double totalMass = shoulderMassKG + coneMassKG;
            double newCOM = (shoulderMassKG * getCOMBicepMeters() + coneMassKG * coneDistance) / totalMass;
            SmartDashboard.putNumber("shcom wo cone", shoulderMassKG * getCOMBicepMeters());
            SmartDashboard.putNumber("shcom wt cone", totalMass * newCOM);
            return totalMass * newCOM * GRAV_CONST;
        }
    }

    public double getElbowCosMult() {
        if (Manipulator.isOpen) {
            return elbowCosineMultiplierNoCOM * getCOMForearmMeters();
        } else {
            double coneDistance = Units.inchesToMeters(26+11);
            double totalMass = elbowMassKG + coneMassKG;
            double newCOM = (elbowMassKG * getCOMForearmMeters() + coneMassKG * coneDistance) / totalMass;
            SmartDashboard.putNumber("elcom wo cone", elbowMassKG * getCOMForearmMeters());
            SmartDashboard.putNumber("elcom wt cone", totalMass * newCOM);
            return totalMass * newCOM * GRAV_CONST;
        }
    }

    public double getWristCosMult() {
        if (Manipulator.isOpen) {
            return wristCosineMultiplierNoCOM * wristCOMMeters;
        } else {
            double coneDistance = Units.inchesToMeters(11);
            double totalMass = wristMassKG + coneMassKG;
            double newCOM = (wristMassKG * wristCOMMeters + coneMassKG * coneDistance) / totalMass;
            SmartDashboard.putNumber("wrcom wo cone", wristMassKG * wristCOMMeters);
            SmartDashboard.putNumber("wrcom wt cone", totalMass * newCOM);
            return totalMass * newCOM * GRAV_CONST;
        }
    }

    private final List<CANSparkMax> sparks;

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

    public Arm(CANSparkMax outerArmLeft, CANSparkMax outerArmRight, CANSparkMax innerArmLeft, CANSparkMax innerArmRight, CANSparkMax wrist) {
        outerArmLeft.setInverted(false);
        outerArmRight.follow(outerArmLeft, true);
        outerArmLeft.setIdleMode(IdleMode.kBrake);
        outerArmRight.setIdleMode(IdleMode.kBrake);

        innerArmLeft.setInverted(true);
        innerArmRight.follow(innerArmLeft, true);
        innerArmLeft.setIdleMode(IdleMode.kBrake);
        innerArmRight.setIdleMode(IdleMode.kBrake);

        innerArmLeft.enableSoftLimit(SoftLimitDirection.kForward, false);
        innerArmLeft.enableSoftLimit(SoftLimitDirection.kReverse, false);

        // innerArm.setSoftLimit(SoftLimitDirection.kForward, 0.80f);
        // innerArm.setSoftLimit(SoftLimitDirection.kReverse, 0.20f);

        wrist.setInverted(true);
        wrist.setIdleMode(IdleMode.kBrake);

        sparks = List.of(outerArmLeft, outerArmRight, innerArmLeft, innerArmRight, wrist);

        sparks.forEach((spark) -> {
            spark.enableVoltageCompensation(12.0);
            spark.setSmartCurrentLimit(40, 30);
            spark.burnFlash();
        });

        ComponentCalculationConstants outerCCC = new ComponentCalculationConstants(
            SHOULDER_TORQUE_TOTAL,
            shoulderMassKG,
            shoulderMinCOMMeters,
            shoulderMaxCOMMeters,
            0.0,
            Units.inchesToMeters(34),
            1,
            MAX_SPEED_OUTER,
            this.goalPose::getNorm,
            (setpoint) -> calcBicepAngleToGround(setpoint),
            () -> -getInner().getPositionRadians()
        );
        ComponentCalculationConstants innerCCC = new ComponentCalculationConstants(
            ELBOW_TORQUE_TOTAL,
            elbowMassKG,
            elbowMinCOMMeters,
            elbowMaxCOMMeters,
            0.0,
            Units.inchesToMeters(26),
            -1,
            MAX_SPEED_INNER,
            () -> Units.inchesToMeters(26),
            (setpoint) -> calcForearmAngleToGround(bicepAngle, setpoint),
            () -> getWrist().getPositionRadians()
        );
        ComponentCalculationConstants wristCCC = new ComponentCalculationConstants(
            WRIST_TORQUE_TOTAL,
            wristMassKG,
            wristCOMMeters,
            wristCOMMeters,
            wristCOMMeters,
            Units.inchesToMeters(13),
            1,
            MAX_SPEED_WRIST,
            () -> 0.0,
            (setpoint) -> calcHandAngleToGround(bicepAngle, forearmAngle, setpoint),
            () -> 0.0
        );

        wristComponent = new ArmComponent(wrist, new MotionMagic(0.15, 0.0, 0.0, 0.0), wristCCC, (component) -> {
            Rotation2d wa2g = calcHandAngleToGround(bicepAngle, forearmAngle, fromRadians(component.getSetpointRadians()));
            double arbFF = getWristCosMult() * wa2g.getCos() / WRIST_TORQUE_TOTAL;

            component.getController().setReference(component.getSetpointRadians(), ControlType.kPosition, 0, arbFF, ArbFFUnits.kPercentOut);
            SmartDashboard.putNumber("wrist set", component.getSetpointRadians());
            SmartDashboard.putNumber("wrist arbff", arbFF);
        });

        innerComponent = new ArmComponent(innerArmLeft, innerArmRight, new MotionMagic(1.0, 0.0, 0.0, 0.0), innerCCC, (component) -> {
            Rotation2d fa2g = calcForearmAngleToGround(bicepAngle, fromRadians(component.getSetpointRadians()));
            double arbFF = -getElbowCosMult() * fa2g.getCos() / ELBOW_TORQUE_TOTAL;

            component.getController().setReference(component.getSetpointRadians(), ControlType.kPosition, 0, arbFF, ArbFFUnits.kPercentOut);
            SmartDashboard.putNumber("elbow set", component.getSetpointRadians());
            SmartDashboard.putNumber("elbow arbff", arbFF);
        });

        outerComponent = new ArmComponent(outerArmLeft, outerArmRight, new MotionMagic(0.5, 0.0, 0.0, 0.0), outerCCC, (component) -> {
            Rotation2d ba2g = calcBicepAngleToGround(fromRadians(component.getSetpointRadians()));
            double arbFF = getShoulderCosMult() * ba2g.getCos() / SHOULDER_TORQUE_TOTAL;

            component.getController().setReference(component.getSetpointRadians(), ControlType.kPosition, 0, arbFF, ArbFFUnits.kPercentOut);
            SmartDashboard.putNumber("shoulder set", component.getSetpointRadians());
            SmartDashboard.putNumber("shoulder arbff", arbFF);
        });
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

    public void setGoal(Translation2d v) {
        goalPose = v;
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

    public double getCOMBicepMeters() {
        double mapped = -innerComponent.getPositionRadians();
        return Calc.map(Math.cos(mapped), 1, -1, shoulderMaxCOMMeters, shoulderMinCOMMeters);
    }

    public double getCOMForearmMeters() {
        double mapped = wristComponent.getPositionRadians();
        return Calc.map(Math.cos(mapped), 1, -1, elbowMaxCOMMeters, elbowMinCOMMeters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("shoulder spd", outerComponent.getMotor().getAppliedOutput());
        SmartDashboard.putNumber("elbow spd", innerComponent.getMotor().getAppliedOutput());
        SmartDashboard.putNumber("wrist spd", wristComponent.getMotor().getAppliedOutput());

        bicepAngle = fromRadians(outerComponent.getEncoder().getPosition());
        forearmAngle = fromRadians(innerComponent.getEncoder().getPosition());
        handAngle = fromRadians(wristComponent.getEncoder().getPosition());

        bicepAngleToGround = calcBicepAngleToGround(bicepAngle);
        forearmAngleToGround = calcForearmAngleToGround(bicepAngle, forearmAngle);
        handAngleToGround = calcHandAngleToGround(bicepAngle, forearmAngle, handAngle);

        SmartDashboard.putNumber("bicep angle", bicepAngle.getDegrees());
        SmartDashboard.putNumber("bicep angle to ground", bicepAngleToGround.getDegrees());
        SmartDashboard.putNumber("forearm angle", forearmAngle.getDegrees());
        SmartDashboard.putNumber("forearm angle to ground", forearmAngleToGround.getDegrees());
        SmartDashboard.putNumber("hand angle", handAngle.getDegrees());
        SmartDashboard.putNumber("hand angle to ground", handAngleToGround.getDegrees());

        SmartDashboard.putNumber("shoulder cur", bicepAngle.getRadians());
        SmartDashboard.putNumber("elbow cur", forearmAngle.getRadians());
        SmartDashboard.putNumber("wrist cur", handAngle.getDegrees());
        SmartDashboard.putBoolean("shoulder atSetpoint", outerComponent.atSetpoint());
        SmartDashboard.putBoolean("elbow atSetpoint", innerComponent.atSetpoint());
        SmartDashboard.putBoolean("wrist atSetpoint", wristComponent.atSetpoint());

        SmartDashboard.putNumber("wrist error", Math.abs(wristComponent.absoluteEncoder.getPosition() - wristComponent.setpoint));

        solveKinematics(goalPose);
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
            this);
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
            this);
    }

    public void setGoalToCurrentPosition() {
        Translation2d newGoal = solver.anglesToPos(outerComponent.getPositionRadians(), innerComponent.getPositionRadians());
        setGoal(newGoal);
    }

    public void solveKinematics(Translation2d goal) {
        Pair<Double, Double> ik = solver.posToAngles(goal);
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

    public class ArmComponent {
        private final CANSparkMax motor;
        private final Optional<CANSparkMax> follow;
        private final SparkMaxPIDController controller;
        private final AbsoluteEncoder absoluteEncoder;
        private final Consumer<ArmComponent> setter;
        public final double MAX_SPEED;
        public final ComponentCalculationConstants ccc;

        public String jointName;
        public String segmentName;

        private double setpoint;

        public static record ComponentCalculationConstants(
            double totalTorqueNM,
            double massKG,
            double minCOMMeters,
            double maxCOMMeters,
            double metersToCOM,
            double segmentLengthMeters,
            int directionMultiplier,
            double maxSpeed,
            Supplier<Double> metersToWrist,
            Function<Rotation2d, Rotation2d> angleToGroundOfSetpoint,
            Supplier<Double> normalizedChildAngle
        ) { }

        public ArmComponent(CANSparkMax motor, CANSparkMax follow, MotionMagic pidConstants, ComponentCalculationConstants ccc, Consumer<ArmComponent> setter) {
            this.motor = motor;
            this.follow = Optional.ofNullable(follow);

            controller = motor.getPIDController();
            absoluteEncoder = motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
            this.setter = setter;
            // this.setter = this::newSetter;
            this.ccc = ccc;
            this.MAX_SPEED = ccc.maxSpeed();

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

        public ArmComponent(CANSparkMax motor, MotionMagic pidConstants, ComponentCalculationConstants ccc, Consumer<ArmComponent> setter) {
            this(motor, null, pidConstants, ccc, setter);
        }

        public void setNames(String jointName, String segmentName) {
            this.jointName = jointName;
            this.segmentName = segmentName;
        }

        public void updateDashboard() {
            if(!jointName.isEmpty()) {
                SmartDashboard.putNumber(jointName + " goal angle", getSetpointDegrees());
                SmartDashboard.putNumber(jointName + " encoder angle", getPositionDegrees());
                SmartDashboard.putBoolean(jointName + " atSetpoint", this.atSetpoint());
            }
        }


        // Calculation

        private double getCOMMeters() {
            double mapped = ccc.normalizedChildAngle.get();
            return Calc.map(Math.cos(mapped), 1, -1, ccc.maxCOMMeters, ccc.minCOMMeters);
        }

        private double getCosMult() {
            if (Manipulator.isOpen) {
                return ccc.massKG * getCOMMeters() * GRAV_CONST;
            } else {
                double coneDistance = ccc.metersToWrist.get() + metersFromWristToGamePiece;
                double totalMass = ccc.massKG + coneMassKG;
                double newCOM = (ccc.massKG * getCOMMeters() + coneMassKG * coneDistance) / totalMass;
                if (!jointName.isEmpty()) {
                    SmartDashboard.putNumber(jointName.substring(0, 2)+"com wo cone", ccc.massKG * getCOMMeters());
                    SmartDashboard.putNumber(jointName.substring(0, 2)+"com wt cone", totalMass * newCOM);
                }
                return totalMass * newCOM * GRAV_CONST;
            }
        }

        private void newSetter() {
            Rotation2d a2g = ccc.angleToGroundOfSetpoint.apply(fromRadians(getSetpointRadians()));
            double arbFF = getCosMult() * a2g.getCos() / ccc.totalTorqueNM;

            getController().setReference(getSetpointRadians(), ControlType.kPosition, 0, arbFF, ArbFFUnits.kPercentOut);
            if (!jointName.isEmpty()) {
                SmartDashboard.putNumber(jointName+" nset", getSetpointRadians());
                SmartDashboard.putNumber(jointName+" narbff", arbFF);
            }
        }

        // End Calculation


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


        public void setDegrees(double value) {
            this.setRadians(degreesToRadians(value));
        }

        public void setRadians(double value) {
            setpoint = value;
            setter.accept(this);
            newSetter();
        }

        public void add(double degrees) {
            setDegrees(getSetpointDegrees() + degrees);
        }

        public void setSpeed(double value) {
            double modSpd = MathUtil.clamp(value, -1, 1);
            motor.set(modSpd * MAX_SPEED);
        }

        public boolean atSetpoint() {
            return Math.abs(absoluteEncoder.getPosition() - MathUtil.inputModulus(setpoint, 0, 2*Math.PI)) < SETPOINT_POSITION_TOLERANCE 
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

        // Will currently require all of the arm, so no other command can
        // be scheduled in parallel
        // TODO make ArmComponent into its own SubsystemBase
        public Command setDegreesCommand(double degrees) {
            return runOnce(() -> this.setDegrees(degrees))
                    .andThen(Commands.waitUntil(this::atSetpoint));
        }

        public Command setRadiansCommand(double radians) {
            return runOnce(() -> this.setRadians(radians))
                    .andThen(Commands.waitUntil(this::atSetpoint));
        }
    }
}
