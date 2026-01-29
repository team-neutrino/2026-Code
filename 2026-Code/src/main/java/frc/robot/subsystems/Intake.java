package frc.robot.subsystems;

import static frc.robot.util.Constants.IntakeConstants.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.RioConstants;

import static frc.robot.util.Subsystems2026.*;

public class Intake extends SubsystemBase {
    private final CANBus m_CANbus = RioConstants.RIO_BUS;
    private TalonFX m_rollerMotor = new TalonFX(ROLLER_MOTOR_ID, m_CANbus);
    private TalonFX m_deployMotor = new TalonFX(DEPLOY_MOTOR_ID, m_CANbus);
    private double m_rollerMotorVoltage;
    private double m_deployMotorVoltage;
    private TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();
    private final CurrentLimitsConfigs m_currentLimitConfig = new CurrentLimitsConfigs();

    public Intake() {
        m_currentLimitConfig.withSupplyCurrentLimit(CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true);
        m_motorConfig.CurrentLimits = m_currentLimitConfig;

        m_rollerMotor.getConfigurator().apply(m_motorConfig);
        m_deployMotor.getConfigurator().apply(m_motorConfig);
        m_rollerMotor.setNeutralMode(NeutralModeValue.Coast);
        m_deployMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    @Override
    public void periodic() {
        m_rollerMotor.setVoltage(m_rollerMotorVoltage);
        m_deployMotor.setVoltage(m_deployMotorVoltage);
    }

    public Command runIntake(double speed) {
        return run(() -> {
            m_rollerMotorVoltage = speed;
            index.m_isHopperEmpty = false;
            index.m_hopperCheckTimer.reset();
        });
    }

    public Command deployIntake(double speed) {
        return run(() -> {
            m_deployMotorVoltage = speed;
        });
    }

    public Command defaultCommand() {
        return run(() -> {
            m_rollerMotorVoltage = 0;
            m_deployMotorVoltage = 0;
        });
    }
}
