package frc.robot.subsystems;

import static frc.robot.util.Constants.IntakeConstants.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    private final CANBus m_CANbus = new CANBus("rio");
    private TalonFX m_rollerMotor = new TalonFX(12, m_CANbus);
    private TalonFX m_deployMotor = new TalonFX(13, m_CANbus);
    private double m_rollerMotorVoltage;
    private double m_deployMotorVoltage;
    // private double m_deployedAngle = DEPLOYED_ANGLE;
    // private double m_motorAngle;
    // Both above if encoder used



    @Override
    public void periodic() {
        m_rollerMotor.setVoltage(m_rollerMotorVoltage);
        m_deployMotor.setVoltage(m_deployMotorVoltage);
  
    }

    public Command runIntake(){
        return run(() -> {
            m_rollerMotorVoltage = INTAKE_VOLTAGE;
        });
    }

    public Command runOuttake(){
        return run(() -> {
            m_rollerMotorVoltage = -INTAKE_VOLTAGE;
        });
    }

    public Command deployIntake(){
        return run (() -> {
            m_deployMotorVoltage = DEPLOY_VOLTAGE;
        });
    }

    public Command retractIntake(){
        return run (() -> {
            m_deployMotorVoltage = -DEPLOY_VOLTAGE;
        });
    }

    public Command defaultCommand() {
        return run(() -> {
            m_rollerMotorVoltage = 0;
            m_deployMotorVoltage = 0;
        });
    }
}
