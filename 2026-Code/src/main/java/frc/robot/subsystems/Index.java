package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.RioConstants;

import static frc.robot.util.Constants.IndexerConstants.*;

public class Index extends SubsystemBase {
    private final CANBus m_CANbus = RioConstants.RIO_BUS;
    private TalonFX m_spindexerMotor = new TalonFX(SPINDEXER_MOTOR_ID, m_CANbus);
    private TalonFX m_outputMotor = new TalonFX(OUTPUT_MOTOR_ID, m_CANbus);
    private CANrange m_canrange1 = new CANrange(19, m_CANbus); //probably going to change to beam breaks
    private CANrange m_canrange2 = new CANrange(20, m_CANbus);
    private double m_spindexerMotorVoltage;
    private double m_outputMotorVoltage;
    private TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();
    private final CurrentLimitsConfigs m_currentLimitConfig = new CurrentLimitsConfigs();

    public Index() {
        m_currentLimitConfig.withSupplyCurrentLimit(CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true);
        m_motorConfig.CurrentLimits = m_currentLimitConfig;

        m_spindexerMotor.getConfigurator().apply(m_motorConfig);
        m_spindexerMotor.setNeutralMode(NeutralModeValue.Coast);
        m_outputMotor.getConfigurator().apply(m_motorConfig);
        m_outputMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    private boolean bothInRange(){
        var range1 = m_canrange1.getIsDetected();
        boolean detected1 = range1.getValue();
        var range2 = m_canrange2.getIsDetected();
        boolean detected2 = range2.getValue();
        return detected1 && detected2;
        // Get the distance of CANrange, if ball is in range for long enough, then full
        // Maybe for different one, but check to see if both sensors detect ball
    }

    // private boolean fullCapacity(){
    //     if(bothInRange()){
    //         double startTime = System.currentTimeMillis();
    //     }
    //     if (startTime - System.currentTimeMillis() >= 2) {
    //         //vibrate the controller
    //     }
    // }

    @Override
    public void periodic() {
        m_spindexerMotor.setVoltage(m_spindexerMotorVoltage);
        m_outputMotor.setVoltage(m_outputMotorVoltage);
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
            m_outputMotorVoltage = 0;
        });
    }
}
