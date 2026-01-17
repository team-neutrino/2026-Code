package frc.robot.subsystems;

import static frc.robot.util.Constants.ClimbConstants.*;

import com.ctre.phoenix6.CANBus;
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

public class Climb extends SubsystemBase {

    private final CANBus m_CANbus = new CANBus("rio");
    private TalonFX m_climbMotor = new TalonFX(CLIMB_MOTOR_ID_1, m_CANbus);
    private TalonFXConfiguration m_climbMotorConfig = new TalonFXConfiguration();
    private final CurrentLimitsConfigs m_currentLimitConfig = new CurrentLimitsConfigs();

    private CANrange m_CANRange = new CANrange(CANRANGE_ID, m_CANbus);

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

        m_canandColor.setSettings(m_settings);
    }

    private void moveClimb(double targetPosition) {
        PositionVoltage positionControl = new PositionVoltage(targetPosition);
        m_climbMotor.setControl(positionControl);
    }

    private boolean atTargetPosition() {
        if (getPosition() <= m_climbTargetPosition + ALLOWED_ERROR && getPosition() >= m_climbTargetPosition - ALLOWED_ERROR) {
            return true;
        } else {
            return false;
        }
    }

    private double getPosition() {
        return m_climbMotor.getPosition().getValueAsDouble();
    }

    private double getDistance() {
        return m_CANRange.getDistance().getValueAsDouble();
    }

    private boolean isClimbOverBar() {
        if(m_canandColor.getProximity() <= CANANDCOLOR_DISTANCE){
            return true;
        } else {
            return false;
        }
    }

    public Command moveClimbCommand(double position) {
        return run(() -> {
            m_climbTargetPosition = position;
            m_runClimb = true;
        });
    }
    
    @Override
    public void periodic() {
        if (m_runClimb) {
            moveClimb(m_climbTargetPosition);
            if(atTargetPosition()){
                m_runClimb = false;
            }
        }
    }
}
