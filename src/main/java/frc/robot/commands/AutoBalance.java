package frc.robot.commands;

import java.util.function.DoubleSupplier;
import com.ctre.phoenix.Logger;
import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drivebase;

public class AutoBalance extends CommandBase{
    public final Drivebase m_drivetrain;
    public Pigeon2 m_piggy = new Pigeon2(0);
    public BangBangController m_autoAlignBangBang = new BangBangController();
    public final Double m_desiredDegrees;
    public double m_piggyRotation = 0;
    public SimpleMotorFeedforward m_bangBangFeedforward = new SimpleMotorFeedforward(2, 1);



    public AutoBalance(SimpleMotorFeedforward bangBangFeedforward, Drivebase drivetrain, Pigeon2 piggy, BangBangController autoAlignBangBang, Double desiredDegrees){        
            this.m_drivetrain = drivetrain;
            this.m_piggy = piggy;
            this.m_autoAlignBangBang = autoAlignBangBang;
            this.m_desiredDegrees = desiredDegrees;
            this.m_bangBangFeedforward = bangBangFeedforward;
    } 

    @Override
    public void initialize(){
        System.out.println("AutoBalance begin ");
    }

    @Override
    public void execute(){
        m_piggyRotation = m_piggy.getPitch();   
        while(m_piggyRotation != 0){
            if(m_piggyRotation < 0){
                m_drivetrain.setVoltage(m_autoAlignBangBang.calculate(m_piggy.getPitch(), 0) * 2 + 0.25 * m_bangBangFeedforward.calculate(0));
            } else if (m_piggyRotation > 0){
                m_drivetrain.setVoltage(m_autoAlignBangBang.calculate(m_piggy.getPitch(), 0) * 2 + 0.25 * m_bangBangFeedforward.calculate(0));
            }   else {
                //go to end
            }
        }

    }

    @Override
    public boolean isFinished(){
        if(0 == m_piggyRotation){
            return true;
        } else {
            return false;
        }
    }

}