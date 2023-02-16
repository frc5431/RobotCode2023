package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.subsystems.*;
import frc.team5431.titan.core.solenoid.DoubleSolenoid;
import frc.team5431.titan.core.solenoid.SingleSolenoid;

public class Systems {
    private Drivebase drivebase;
    private Vision vision;

    private Arm arm;
    private Manipulator manipulator;

    private CANSparkMax armOuterLeft;
    private CANSparkMax armOuterRight;
    private CANSparkMax armInnerLeft;
    private CANSparkMax armInnerRight;
    private CANSparkMax wrist;
    private CANSparkMax intakeLeft;

    private DoubleSolenoid paddles;
    private DoubleSolenoid dblSol2;
    private SingleSolenoid sglSol1;

    private Compressor compressor;
    private PneumaticHub phub;

    public Systems() {
        drivebase = new Drivebase();
        // vision = new Vision(drivebase);

        armOuterLeft = new CANSparkMax(17, MotorType.kBrushless);
        armOuterRight = new CANSparkMax(18, MotorType.kBrushless);
        armInnerLeft = new CANSparkMax(16, MotorType.kBrushless);
        armInnerRight = new CANSparkMax(15, MotorType.kBrushless);
        wrist = new CANSparkMax(19, MotorType.kBrushless);
        arm = new Arm(armOuterLeft, armOuterRight, armInnerLeft, armInnerRight, wrist);

        paddles = new DoubleSolenoid(Constants.ID_PHUB, PneumaticsModuleType.REVPH, 10, 11);
        manipulator = new Manipulator(paddles);

        // intakeLeft = new CANSparkMax(20, MotorType.kBrushless);

        dblSol2 = new DoubleSolenoid(Constants.ID_PHUB, PneumaticsModuleType.REVPH, 8, 9);
        sglSol1 = new SingleSolenoid(Constants.ID_PHUB, PneumaticsModuleType.REVPH, 5);

        paddles.set(DoubleSolenoid.Value.kForward);
        dblSol2.set(DoubleSolenoid.Value.kForward);

        compressor = new Compressor(Constants.ID_PHUB, PneumaticsModuleType.REVPH);
        phub = new PneumaticHub(Constants.ID_PHUB);

        // compressor.enableDigital();
        compressor.enableAnalog(60, 120);
        // compressor.enableHybrid(60, 120);
        // compressor.disable();
    }

    public Drivebase getDrivebase() {
        return drivebase;
    }

    public Vision getVision() {
        return vision;
    }

    public Arm getArm() {
        return arm;
    }

    public Manipulator getManipulator() {
        return manipulator;
    }

    public DoubleSolenoid getDblSol2() {
        return dblSol2;
    }

    public SingleSolenoid getSglSol1() {
        return sglSol1;
    }

    public Compressor getCompressor() {
        return compressor;
    }

    public PneumaticHub getPneumaticHub() {
        return phub;
    }

    public CANSparkMax getIntakeLeft() {
        return intakeLeft;
    }
}
