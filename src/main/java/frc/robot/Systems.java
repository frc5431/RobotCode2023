package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.subsystems.*;
import frc.team5431.titan.core.solenoid.DoubleSolenoid;
import frc.team5431.titan.core.solenoid.SingleSolenoid;

public class Systems {
    private Drivebase drivebase;

    private Arm arm;

    private CANSparkMax armInnerLeft;
    private CANSparkMax armInnerRight;
    private CANSparkMax armOuterLeft;
    private CANSparkMax armOuterRight;

    private DoubleSolenoid dblSol1;
    private DoubleSolenoid dblSol2;
    private SingleSolenoid sglSol1;
    private Vision vision;


    public Systems() {
        drivebase = new Drivebase();
        // vision = new Vision(drivebase);

        armInnerLeft = new CANSparkMax(15, MotorType.kBrushless);
        armInnerRight = new CANSparkMax(16, MotorType.kBrushless);
        armOuterLeft = new CANSparkMax(18, MotorType.kBrushless);
        armOuterRight = new CANSparkMax(17, MotorType.kBrushless);
        arm = new Arm(armOuterLeft, armOuterRight, armInnerLeft, armInnerRight);

        dblSol1 = new DoubleSolenoid(1, PneumaticsModuleType.REVPH, 1, 14);
        dblSol2 = new DoubleSolenoid(1, PneumaticsModuleType.REVPH, 0, 15);
        sglSol1 = new SingleSolenoid(1, PneumaticsModuleType.REVPH, 13);

        dblSol1.set(DoubleSolenoid.Value.kForward);
        dblSol2.set(DoubleSolenoid.Value.kForward);

        try (Compressor compressor = new Compressor(1, PneumaticsModuleType.REVPH)) {
            // compressor.enableDigital();
            // compressor.enableHybrid(60, 120);
            compressor.disable();
        }
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

    public DoubleSolenoid getDblSol1() {
        return dblSol1;
    }

    public DoubleSolenoid getDblSol2() {
        return dblSol2;
    }

    public SingleSolenoid getSglSol1() {
        return sglSol1;
    }
}
