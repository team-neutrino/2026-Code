package frc.robot.subsystems;

import static frc.robot.util.Constants.ClimbConstants.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.RioConstants;

public class Climb extends SubsystemBase {

    private final CANBus m_CANbus = RioConstants.RIO_BUS;
    private TalonFX m_climbMotor = new TalonFX(CLIMB_MOTOR_ID, m_CANbus);
    private TalonFXConfiguration m_climbMotorConfig = new TalonFXConfiguration();
    private final CurrentLimitsConfigs m_currentLimitConfig = new CurrentLimitsConfigs();

    private CANrange m_CANRange = new CANrange(CANRANGE_ID, m_CANbus);
    private CANrangeConfiguration m_CANRangeConfiguration = new CANrangeConfiguration();

    private double m_climbTargetPosition = 0;
    private boolean m_runClimb = false;
    private int m_slot = 0;

    public Climb() {
        m_currentLimitConfig.withSupplyCurrentLimit(CLIMB_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(CLIMB_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true);
        m_climbMotorConfig.CurrentLimits = m_currentLimitConfig;

        m_climbMotorConfig.Slot0.kP = CLIMB_kP_1;
        m_climbMotorConfig.Slot0.kI = CLIMB_kI_1;
        m_climbMotorConfig.Slot0.kD = CLIMB_kD_1;

        m_climbMotorConfig.Slot1.kP = CLIMB_kP_2;
        m_climbMotorConfig.Slot1.kI = CLIMB_kI_2;
        m_climbMotorConfig.Slot1.kD = CLIMB_kD_2;

        m_climbMotor.getConfigurator().apply(m_climbMotorConfig);
        m_climbMotor.setNeutralMode(NeutralModeValue.Brake);

        m_climbMotor.setPosition(0);

        m_CANRangeConfiguration.ProximityParams.ProximityThreshold = CANRANGE_THRESHOLD;
        m_CANRangeConfiguration.ProximityParams.ProximityHysteresis = CANRANGE_HYSTERSIS;
        m_CANRange.getConfigurator().apply(m_CANRangeConfiguration);
    }

    private void moveClimb(double targetPosition) {
        PositionVoltage positionControl = new PositionVoltage(targetPosition).withSlot(m_slot);
        positionControl.FeedForward = CLIMB_kFF;
        m_climbMotor.setControl(positionControl);
    }

    public boolean atTargetPosition() {
        return Math.abs(getClimbPosition() - m_climbTargetPosition) <= ALLOWED_ERROR;
    }

    public double getClimbPosition() {
        return m_climbMotor.getPosition().getValueAsDouble();
    }

    public double getClimbTargetPosition() {
        return m_climbTargetPosition;
    }

    public double getCANRangeDistance() {
        return m_CANRange.getDistance().getValueAsDouble();
    }

    public boolean isCANRangeDetected() {
        return m_CANRange.getIsDetected().getValue();
    }

    public boolean atCANRangeClimbPosition() {
        return getCANRangeDistance() <= ALLOWED_ERROR;
    }

    public void setClimbPID(double new_P, double new_I, double new_D, long slot) {
        if (slot == 1) {
            m_climbMotorConfig.Slot1.kP = new_P;
            m_climbMotorConfig.Slot1.kI = new_I;
            m_climbMotorConfig.Slot1.kD = new_D;
        } else {
            m_climbMotorConfig.Slot0.kP = new_P;
            m_climbMotorConfig.Slot0.kI = new_I;
            m_climbMotorConfig.Slot0.kD = new_D;
        }

        m_climbMotor.getConfigurator().apply(m_climbMotorConfig);
    }

    public Command moveClimbCommand(double position, int slot) {
        return run(() -> {
            m_climbTargetPosition = position;
            m_slot = slot;
            m_runClimb = true;
        });
    }

    public Command defaultClimbCommand() {
        return run(() -> {
            m_climbMotor.setVoltage(0);
            m_runClimb = false;
        });
    }

    @Override
    public void periodic() {
        if (m_runClimb) {
            moveClimb(m_climbTargetPosition);
            if (atTargetPosition()) {
                m_runClimb = false;
            }
        }
    }
}