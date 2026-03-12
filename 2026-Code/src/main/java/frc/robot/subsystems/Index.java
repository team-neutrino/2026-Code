package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.reduxrobotics.sensors.canandcolor.CanandcolorSettings;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Timer;
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

    private Canandcolor m_canandColor = new Canandcolor(CANANDCOLOR_ID);
    private CanandcolorSettings m_settings = new CanandcolorSettings();
    private boolean m_emptyHopper = true;
    private Debouncer m_emptyDebouncer = new Debouncer(DEBOUNCED_TIME, DebounceType.kRising);
    private double m_ballsPerSecondCount;
    private boolean m_ballDetected = false;
    public Timer m_bpsTimer = new Timer();
    private final VoltageOut m_spindexerVoltageControl = new VoltageOut(0);
    private final VoltageOut m_kickerVoltageControl = new VoltageOut(0);

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

        m_canandColor.setSettings(m_settings);
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

    public void checkEmptyHopper() {
        if (canandColorDetect()) {
            m_emptyHopper = false;
        } else if (m_emptyDebouncer.calculate(!canandColorDetect())) {
            m_emptyHopper = true;
        }
    }

    public boolean isHopperEmpty() {
        return m_emptyHopper;
    }

    @Override
    public void periodic() {
        checkEmptyHopper();
        calculateBallsPerSecond();
        setSpindexerVoltage();
        setKickerVoltage();
    }

    public void setSpindexerVoltage() {
        m_spindexerVoltageControl.EnableFOC = true;
        m_spindexerMotor.setControl(m_spindexerVoltageControl.withOutput(m_spindexerMotorVoltage));
    }

    public void setKickerVoltage() {
        m_kickerVoltageControl.EnableFOC = true;
        m_kickerMotor.setControl(m_kickerVoltageControl.withOutput(m_kickerMotorVoltage));
    }

    public Command noKickAndSpin() {
        return run(() -> {
            m_kickerMotorVoltage = 0;
            m_spindexerMotorVoltage = 0;
        });
    }

    public Command feedShooter() {
        return run(() -> {
            m_spindexerMotorVoltage = INDEXING_VOLTAGE;
            m_kickerMotorVoltage = KICKER_VOLTAGE;
        });
    }

    public Command defaultCommand() {
        return run(() -> {
            if (shooterArbiter.readyToFire()) {
                m_spindexerMotorVoltage = INDEXING_VOLTAGE;
                m_kickerMotorVoltage = KICKER_VOLTAGE;
            } else {
                m_spindexerMotorVoltage = 0.0;
                m_kickerMotorVoltage = 0.0;
            }
        });
    }
}
