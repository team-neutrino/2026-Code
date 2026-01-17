package frc.robot.subsystems;

import static frc.robot.util.Constants.IndexerConstants.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase{
    private final CANBus m_CANbus = new CANBus("rio");
    private TalonFX m_motor1 = new TalonFX(MOTOR_1_ID, m_CANbus);
    private TalonFX m_motor2 = new TalonFX(MOTOR_2_ID, m_CANbus);
    private double m_motor1Voltage;
    private double m_motor2Voltage;
    private TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();
    private final CurrentLimitsConfigs m_currentLimitConfig = new CurrentLimitsConfigs();

    public Indexer() {
        m_currentLimitConfig.withSupplyCurrentLimit(CURRENT_LIMIT)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(CURRENT_LIMIT)
            .withStatorCurrentLimitEnable(true);
        m_motorConfig.CurrentLimits = m_currentLimitConfig;

        m_motor1.getConfigurator().apply(m_motorConfig);
        m_motor2.getConfigurator().apply(m_motorConfig);
        m_motor1.setNeutralMode(NeutralModeValue.Coast);
        m_motor2.setNeutralMode(NeutralModeValue.Coast);
    }

    @Override
    public void periodic() {
        m_motor1.setVoltage(m_motor1Voltage);
        m_motor2.setVoltage(m_motor2Voltage);
    }

    public Command runIndexer(double speed){
        return run (() -> {
            m_motor1Voltage = speed;
            m_motor2Voltage = speed;
        });
    }

    public Command defaultCommand() {
        return run(() -> {
            m_motor1Voltage = 0;
            m_motor2Voltage = 0;
        });
    }
}

