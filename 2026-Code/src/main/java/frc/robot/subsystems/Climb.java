package frc.robot.subsystems;

import static frc.robot.util.Constants.ClimbConstants.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfigurator;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.reduxrobotics.sensors.canandcolor.CanandcolorSettings;

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

    private Canandcolor m_canandColor = new Canandcolor(CANANDCOLOR_ID);
    private CanandcolorSettings m_settings = new CanandcolorSettings();

    private double m_climbTargetPosition = 0;
    private boolean m_runClimb = false;

    public Climb() {
        m_currentLimitConfig.withSupplyCurrentLimit(CLIMB_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(CLIMB_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true);
        m_climbMotorConfig.CurrentLimits = m_currentLimitConfig;

        m_climbMotorConfig.Slot0.kP = CLIMB_kP;
        m_climbMotorConfig.Slot0.kI = CLIMB_kI;
        m_climbMotorConfig.Slot0.kD = CLIMB_kD;

        m_climbMotor.getConfigurator().apply(m_climbMotorConfig);
        m_climbMotor.setNeutralMode(NeutralModeValue.Brake);

        m_climbMotor.setPosition(0);

        m_canandColor.setSettings(m_settings);

        m_CANRangeConfiguration.ProximityParams.ProximityThreshold = 0.6;
        m_CANRangeConfiguration.ProximityParams.ProximityHysteresis = 0.01;
        m_CANRange.getConfigurator().apply(m_CANRangeConfiguration);
    }

    private void moveClimb(double targetPosition) {
        PositionVoltage positionControl = new PositionVoltage(targetPosition);
        positionControl.FeedForward = CLIMB_kFF;
        m_climbMotor.setControl(positionControl);
    }

    public boolean atTargetPosition() {
        return getClimbPosition() <= m_climbTargetPosition + ALLOWED_ERROR
                && getClimbPosition() >= m_climbTargetPosition - ALLOWED_ERROR;
    }

    public double getClimbPosition() {
        return m_climbMotor.getPosition().getValueAsDouble();
    }

    public double getClimbTargetPosition() {
        return m_climbTargetPosition;
    }

    public double getClimbCurrent() {
        return m_climbMotor.getSupplyCurrent().getValueAsDouble();
    }

    public double getCANDistance() {
        return m_CANRange.getDistance().getValueAsDouble();
    }

    public boolean isCANRangeDetected() {
        return m_CANRange.getIsDetected().getValue();
    }

    public boolean isClimbOverBar() {
        return m_canandColor.getProximity() <= CANANDCOLOR_DISTANCE;
    }

    public Command moveClimbCommand(double position) {
        return run(() -> {
            m_climbTargetPosition = position;
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