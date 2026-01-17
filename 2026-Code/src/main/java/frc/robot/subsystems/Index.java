package frc.robot.subsystems;

import static frc.robot.util.Constants.IndexerConstants.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Index extends SubsystemBase{
    private final CANBus m_CANbus = new CANBus("rio");
    private TalonFX m_spindexerMotor = new TalonFX(SPINDEXER_MOTOR_ID, m_CANbus);
    private double m_spindexerMotorVoltage;
    private TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();
    private final CurrentLimitsConfigs m_currentLimitConfig = new CurrentLimitsConfigs();

    public Index() {
        m_currentLimitConfig.withSupplyCurrentLimit(CURRENT_LIMIT)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(CURRENT_LIMIT)
            .withStatorCurrentLimitEnable(true);
        m_motorConfig.CurrentLimits = m_currentLimitConfig;

        m_spindexerMotor.getConfigurator().apply(m_motorConfig);
        m_spindexerMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    @Override
    public void periodic() {
        m_spindexerMotor.setVoltage(m_spindexerMotorVoltage);
    }

    public Command runSpindexer(double speed){
        return run (() -> {
            m_spindexerMotorVoltage = speed;
        });
    }

    public Command defaultCommand() {
        return run(() -> {
            m_spindexerMotorVoltage = 0;
        });
    }
}

