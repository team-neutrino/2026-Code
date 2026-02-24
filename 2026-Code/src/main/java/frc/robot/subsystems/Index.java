package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.RioConstants;

import static frc.robot.util.Constants.IndexConstants.*;
import static frc.robot.util.Subsystems.shooterArbiter;

public class Index extends SubsystemBase {
    private final CANBus m_CANbus = RioConstants.RIO_BUS;

    private TalonFX m_spindexerMotor = new TalonFX(SPINDEXER_MOTOR_ID, m_CANbus);
    private double m_spindexerMotorVoltage;
    private TalonFXConfiguration m_indexMotorConfig = new TalonFXConfiguration();
    private final CurrentLimitsConfigs m_indexCurrentLimitConfig = new CurrentLimitsConfigs();

    private TalonFX m_kickerMotor = new TalonFX(KICKER_MOTOR_ID, m_CANbus);
    private double m_kickerMotorVoltage;
    private TalonFXConfiguration m_kickerMotorConfig = new TalonFXConfiguration();
    private final CurrentLimitsConfigs m_kickerCurrentLimitConfig = new CurrentLimitsConfigs();

    public Index() {
        m_indexCurrentLimitConfig.withSupplyCurrentLimit(INDEX_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(INDEX_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true);
        m_indexMotorConfig.CurrentLimits = m_indexCurrentLimitConfig;

        m_kickerCurrentLimitConfig.withSupplyCurrentLimit(KICKER_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(KICKER_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true);
        m_kickerMotorConfig.CurrentLimits = m_kickerCurrentLimitConfig;

        m_spindexerMotor.getConfigurator().apply(m_indexMotorConfig);
        m_kickerMotor.getConfigurator().apply(m_kickerMotorConfig);

        m_spindexerMotor.setNeutralMode(NeutralModeValue.Coast);
        m_kickerMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    public void setVoltageIfShooting() {
        if (shooterArbiter.readyToFire()) {
            m_spindexerMotorVoltage = INDEXING_VOLTAGE;
            m_kickerMotorVoltage = KICKER_VOLTAGE;
        } else {
            m_spindexerMotorVoltage = 0.0;
            m_kickerMotorVoltage = 0.0;
        }
    }

    public double getSpindexerCurrentVoltage() {
        return m_spindexerMotor.getMotorVoltage().getValueAsDouble();
    }

    public double getSpindexerTargetVoltage() {
        return m_spindexerMotorVoltage;
    }

    public double getKickerCurrentVoltage() {
        return m_kickerMotor.getMotorVoltage().getValueAsDouble();
    }

    public double getKickerTargetVoltage() {
        return m_kickerMotorVoltage;
    }

    @Override
    public void periodic() {
        setVoltageIfShooting();
        m_spindexerMotor.setVoltage(m_spindexerMotorVoltage);
        m_kickerMotor.setVoltage(m_kickerMotorVoltage);
    }

    public Command spinWhenPress() {
        return run(() -> {
            m_spindexerMotorVoltage = INDEXING_VOLTAGE;
        });
    }

    public Command kickWhenPress() {
        return run(() -> {
            m_kickerMotorVoltage = KICKER_VOLTAGE;
        });
    }

    public Command defaultCommand() {
        return run(() -> {
        });
    }
}
