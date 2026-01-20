package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.RioConstants;

import static frc.robot.util.Constants.IndexerConstants.*;

public class Index extends SubsystemBase {
    private final CANBus m_CANbus = RioConstants.RIO_BUS;
    private TalonFX m_spindexerMotor = new TalonFX(SPINDEXER_MOTOR_ID, m_CANbus);
    private double m_spindexerMotorVoltage;
    private TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();
    private final CurrentLimitsConfigs m_currentLimitConfig = new CurrentLimitsConfigs();
    private DigitalInput m_beamBreak1 = new DigitalInput(BEAMBREAK_CHANNEL_1);
    private DigitalInput m_beamBreak2 = new DigitalInput(BEAMBREAK_CHANNEL_2);
    private Debouncer m_debouncer = new Debouncer(DEBOUNCED_TIME, Debouncer.DebounceType.kRising);

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
        return m_beamBreak1.get() && m_beamBreak2.get();
    }

    public boolean fullCapacity() {
        if (m_debouncer.calculate(bothBeamsBroken())){
            return true;
        }
        return false;
    }

    

    @Override
    public void periodic() {
        m_spindexerMotor.setVoltage(m_spindexerMotorVoltage);
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
