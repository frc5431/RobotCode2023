package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.subsystems.*;
import frc.team5431.titan.core.leds.Blinkin;
import frc.team5431.titan.core.leds.BlinkinPattern;

public class Systems {
    private Drivebase drivebase;
    private Vision vision;
    private Blinkin leds;

    private ArmContainer arm;
    private Manipulator manipulator;

    private CANSparkMax armOuterLeft;
    private CANSparkMax armOuterRight;
    private CANSparkMax armInnerLeft;
    private CANSparkMax armInnerRight;
    private CANSparkMax wrist;

    public Systems() {
        drivebase = new Drivebase();
        if(Robot.isReal()) {
            // vision = new Vision(drivebase); 
        }
        leds = new Blinkin(0, BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE);

        manipulator = new Manipulator(new CANSparkMax(20, MotorType.kBrushless));

        armOuterLeft = new CANSparkMax(15, MotorType.kBrushless);
        armOuterRight = new CANSparkMax(16, MotorType.kBrushless);
        armInnerLeft = new CANSparkMax(17, MotorType.kBrushless);
        armInnerRight = new CANSparkMax(18, MotorType.kBrushless);
        wrist = new CANSparkMax(19, MotorType.kBrushless);
        arm = new ArmContainer(manipulator, armOuterLeft, armOuterRight, armInnerLeft, armInnerRight, wrist);

        // compressor = new Compressor(Constants.ID_PHUB, PneumaticsModuleType.REVPH);
        // phub = new PneumaticHub(Constants.ID_PHUB);

        // // compressor.enableDigital();
        // // compressor.enableAnalog(60, 105);
        // // compressor.enableHybrid(60, 120);
        // compressor.disable();
    }

    public Drivebase getDrivebase() {
        return drivebase;
    }

    public Vision getVision() {
        return vision;
    }

    public Blinkin getLeds() {
        return leds;
    }

    public ArmContainer getArm() {
        return arm;
    }

    public Manipulator getManipulator() {
        return manipulator;
    }
}
