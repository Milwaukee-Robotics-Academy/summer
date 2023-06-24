package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private CANSparkMax m_motor = new CANSparkMax(3, MotorType.kBrushed);
    
    public Intake() {
        m_motor.restoreFactoryDefaults();
        m_motor.setIdleMode(IdleMode.kBrake);
    }

    public void in(){
        m_motor.set(0.3);
    }

    public void out(){
        m_motor.set(-0.3);
    }

    public void stop(){
        m_motor.set(0.0);
    }
}
