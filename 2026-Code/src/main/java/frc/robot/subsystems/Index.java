package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.util.Constants.RioConstants;

import static frc.robot.util.Constants.IndexerConstants.*;

import java.net.IDN;

public class Index extends SubsystemBase {
    private final CANBus m_CANbus = RioConstants.RIO_BUS;
    private TalonFX m_spindexerMotor = new TalonFX(SPINDEXER_MOTOR_ID, m_CANbus);
    private double m_spindexerMotorVoltage;
    private TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();
    private final CurrentLimitsConfigs m_currentLimitConfig = new CurrentLimitsConfigs();
    private DigitalInput m_capacityBeamBreak1 = new DigitalInput(BEAMBREAK_CHANNEL_1);
    private DigitalInput m_capacityBeamBreak2 = new DigitalInput(BEAMBREAK_CHANNEL_2);
    private DigitalInput m_emptyBeamBreak = new DigitalInput(BEAMBREAK_CHANNEL_3);
    private Debouncer m_startRumbleDebouncer = new Debouncer(START_RUMBLE_DEBOUNCED_TIME,
            Debouncer.DebounceType.kRising);
    private Debouncer m_stopRumbleDebouncer = new Debouncer(STOP_RUMBLE_DEBOUNCED_TIME, Debouncer.DebounceType.kRising);
    private Debouncer m_emptyDebouncer = new Debouncer(MOTOR_START_TIME, Debouncer.DebounceType.kFalling);
    private CommandGenericHID m_rumbleDriver = new CommandGenericHID(0);
    private CommandGenericHID m_rumbleButtons = new CommandGenericHID(1);

    private boolean m_isHopperEmpty;
    private Timer m_indexTimer = new Timer();

    public Index() {
        m_currentLimitConfig.withSupplyCurrentLimit(CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true);
        m_motorConfig.CurrentLimits = m_currentLimitConfig;
        m_spindexerMotor.getConfigurator().apply(m_motorConfig);
        m_spindexerMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    public boolean bothBeamsBroken() {
        return m_capacityBeamBreak1.get() && !m_capacityBeamBreak2.get();
    }

    public boolean fullCapacity() {
        return m_startRumbleDebouncer.calculate(bothBeamsBroken());
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

    @Override
    public void periodic() {
        m_spindexerMotor.setVoltage(m_spindexerMotorVoltage);
        rumbleControllers();
        stopRumble();
        // boolean motorDebounce = m_emptyDebouncer.calculate(m_emptyBeamBreak.get());
        // if (motorDebounce) {
        // m_spindexerMotor.setVoltage(0);
        // } else {
        // m_spindexerMotorVoltage = INDEXING_VOLTAGE;
        // }
        if (m_emptyBeamBreak.get()) {
            m_isHopperEmpty = false;
            m_spindexerMotor.setVoltage(0);
            m_indexTimer.stop();
            m_indexTimer.reset();
        } else {
            if (m_indexTimer.isRunning() && m_indexTimer.hasElapsed(3)) {
                m_isHopperEmpty = true;
                m_spindexerMotor.setVoltage(0);
            } else {
                m_spindexerMotor.setVoltage(INDEXING_VOLTAGE);
                m_indexTimer.start();
            }
        }
    }

    public Command runSpindexer(double speed) {
        return run(() -> {
            m_spindexerMotorVoltage = speed;
        });
    }

    public boolean isEmpty() {
        return false;
    }

    public Command defaultCommand() {
        return run(() -> {
            m_spindexerMotorVoltage = 0;
        });
    }
}
