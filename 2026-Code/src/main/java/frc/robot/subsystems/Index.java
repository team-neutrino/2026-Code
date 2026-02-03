package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreCANrange;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.reduxrobotics.sensors.canandcolor.CanandcolorSettings;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.util.Constants.RioConstants;

import static frc.robot.util.Constants.IndexerConstants.*;
import static frc.robot.util.Subsystems2026.shooterArbiter;

public class Index extends SubsystemBase {
    private final CANBus m_CANbus = RioConstants.RIO_BUS;
    private TalonFX m_spindexerMotor = new TalonFX(SPINDEXER_MOTOR_ID, m_CANbus);
    private double m_spindexerMotorVoltage;
    private TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();
    private final CurrentLimitsConfigs m_currentLimitConfig = new CurrentLimitsConfigs();

    private CoreCANrange m_canRange1 = new CoreCANrange(CANRANGE_CAN_ID_1, m_CANbus);
    private CoreCANrange m_canRange2 = new CoreCANrange(CANRANGE_CAN_ID_2, m_CANbus);

    private Canandcolor m_canandColor = new Canandcolor(CANANDCOLOR_ID);
    private CanandcolorSettings m_settings = new CanandcolorSettings();

    private Debouncer m_fullCapacityDebouncer = new Debouncer(START_RUMBLE_DEBOUNCED_TIME,
            Debouncer.DebounceType.kRising);
    private Debouncer m_stopRumbleDebouncer = new Debouncer(STOP_RUMBLE_DEBOUNCED_TIME, Debouncer.DebounceType.kRising);
    private Debouncer m_emptyDebouncer1 = new Debouncer(MOTOR_START_TIME, Debouncer.DebounceType.kRising);
    private Debouncer m_emptyDebouncer2 = new Debouncer(MOTOR_STOP_TIME, Debouncer.DebounceType.kRising);

    private CommandGenericHID m_rumbleDriver = new CommandGenericHID(0);
    private CommandGenericHID m_rumbleButtons = new CommandGenericHID(1);

    public boolean m_isHopperEmpty;
    public Timer m_hopperCheckTimer = new Timer();

    public Index() {
        m_currentLimitConfig.withSupplyCurrentLimit(CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true);
        m_motorConfig.CurrentLimits = m_currentLimitConfig;
        m_spindexerMotor.getConfigurator().apply(m_motorConfig);
        m_spindexerMotor.setNeutralMode(NeutralModeValue.Coast);

        m_canandColor.setSettings(m_settings);
    }

    public double getCanRangeDistance(CoreCANrange canRange) {
        return canRange.getDistance().getValueAsDouble();
    }

    public boolean bothCanRangesDetect() {
        return getCanRangeDistance(m_canRange1) < FULL_CAPACITY_DISTANCE
                && getCanRangeDistance(m_canRange2) < FULL_CAPACITY_DISTANCE;
    }

    public boolean fullCapacity() {
        return m_fullCapacityDebouncer.calculate(bothCanRangesDetect());
    }

    public double getCanAndColorDistance() {
        return m_canandColor.getProximity();
    }

    public boolean canandColorDetect() {
        return getCanAndColorDistance() < TOWER_CANANDCOLOR_DISTANCE;
    }

    public boolean isHopperEmpty() {
        return m_isHopperEmpty;
    }

    public void rumbleControllers() {
        if (fullCapacity()) {
            m_rumbleDriver.setRumble(RumbleType.kBothRumble, 0.5);
            m_rumbleButtons.setRumble(RumbleType.kBothRumble, 0.5);
        } else {
            m_rumbleDriver.setRumble(RumbleType.kBothRumble, 0);
            m_rumbleButtons.setRumble(RumbleType.kBothRumble, 0);
        }
    }

    public void stopRumble() {
        if (m_stopRumbleDebouncer.calculate(fullCapacity())) {
            m_rumbleButtons.setRumble(RumbleType.kBothRumble, 0);
            m_rumbleDriver.setRumble(RumbleType.kBothRumble, 0);
        }
    }

    public void checkEmptyHopper() {
        boolean motorStartDebounce = m_emptyDebouncer1.calculate(!canandColorDetect());
        boolean motorStopDebounce = m_emptyDebouncer2.calculate(canandColorDetect());
        if (motorStopDebounce) {
            m_isHopperEmpty = false;
            m_spindexerMotorVoltage = 0;
            m_hopperCheckTimer.stop();
            m_hopperCheckTimer.reset();
        } else {
            if (m_hopperCheckTimer.isRunning() &&
                    m_hopperCheckTimer.hasElapsed(HOPPER_CHECK_TIME)) {
                m_isHopperEmpty = true;
                m_spindexerMotorVoltage = 0;
            } else if (motorStartDebounce) {
                m_spindexerMotorVoltage = HOPPER_CHECK_VOLTAGE;
                m_hopperCheckTimer.start();
            }
        }
        if (fullCapacity()) {
            m_isHopperEmpty = false;
        }
        m_spindexerMotor.setVoltage(m_spindexerMotorVoltage);
    }

    @Override
    public void periodic() {
        rumbleControllers();
        stopRumble();
        checkEmptyHopper();
    }

    public Command runSpindexer(double speed) {
        return run(() -> {

        });
    }

    public Command defaultCommand() {
        return run(() -> {
            if (shooterArbiter.readyToFire() || !m_hopperCheckTimer.isRunning()) {
                m_spindexerMotorVoltage = INDEXING_VOLTAGE;
            } else {
                m_spindexerMotorVoltage = 0;
            }
        });
    }
}
