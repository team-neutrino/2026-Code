package frc.robot.subsystems;

import static frc.robot.util.Constants.IntakeConstants.*;

import java.io.ObjectInputFilter.Status;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.RioConstants;

public class Intake extends SubsystemBase {
    private final CANBus m_CANbus = RioConstants.RIO_BUS;
    private TalonFX m_rollerMotor = new TalonFX(ROLLER_MOTOR_ID, m_CANbus);
    private TalonFX m_deployMotor = new TalonFX(DEPLOY_MOTOR_ID, m_CANbus);
    private double m_rollerMotorVoltage;
    private TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();
    private final CurrentLimitsConfigs m_currentLimitConfig = new CurrentLimitsConfigs();
    private double m_targetAngle = STARTING_POSITION;

    public Intake() {
        m_currentLimitConfig.withSupplyCurrentLimit(CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true);
        m_motorConfig.CurrentLimits = m_currentLimitConfig;

        m_motorConfig.Slot0.kP = 0.2; // placeholder value (definitely should be tuned)
        m_motorConfig.Slot0.kI = 0;
        m_motorConfig.Slot0.kD = 0;

        m_rollerMotor.getConfigurator().apply(m_motorConfig);
        m_deployMotor.getConfigurator().apply(m_motorConfig);
        m_rollerMotor.setNeutralMode(NeutralModeValue.Coast);
        m_deployMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    public double getMotorAngle() {
        return m_deployMotor.getPosition().getValueAsDouble();
    }

    public double getTargetAngle() {
        return m_targetAngle;
    }

    private void moveToIntake(double targetPosition) {
        PositionVoltage positionControl = new PositionVoltage(targetPosition);
        m_deployMotor.setControl(positionControl);
    }

    @Override
    public void periodic() {
        m_rollerMotor.setVoltage(m_rollerMotorVoltage);
        moveToIntake(m_targetAngle);
    }

    public Command runIntake(double speed) {
        return run(() -> {
            m_rollerMotorVoltage = speed;
        });
    }

    public Command deployIntake(double targetAngle) {
        return run(() -> {
            m_targetAngle = targetAngle;
        });
    }

    public Command defaultCommand() {
        return run(() -> {
            m_rollerMotorVoltage = 0;
            m_targetAngle = STARTING_POSITION;
        });
    }
}
