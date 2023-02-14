package frc.robot.subsystems.simsystems;

import com.revrobotics.CANSparkMax;

import frc.robot.subsystems.Arm;

public class ArmSim extends Arm {

    public ArmSim(CANSparkMax outerArmLeft, CANSparkMax outerArmRight, CANSparkMax innerArmLeft,
            CANSparkMax innerArmRight, CANSparkMax wrist) {
        super(outerArmLeft, outerArmRight, innerArmLeft, innerArmRight, wrist);
        //TODO Auto-generated constructor stub
    }


    
}
