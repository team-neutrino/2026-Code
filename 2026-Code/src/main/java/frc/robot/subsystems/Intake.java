package frc.robot.subsystems;

import static frc.robot.util.Constants.IntakeConstants.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.RioConstants;

public class Intake extends SubsystemBase {
    private final CANBus m_CANbus = RioConstants.RIO_BUS;
    private TalonFX m_rollerMotor = new TalonFX(ROLLER_MOTOR_ID, m_CANbus);
    private TalonFX m_deployMotor = new TalonFX(DEPLOY_MOTOR_ID, m_CANbus);
    private double m_rollerMotorVoltage;
    private TalonFXConfiguration m_rollerMotorConfig = new TalonFXConfiguration();
    private TalonFXConfiguration m_deployMotorConfig = new TalonFXConfiguration();
    private final CurrentLimitsConfigs m_rollerCurrentLimitConfig = new CurrentLimitsConfigs();
    private final CurrentLimitsConfigs m_deployCurrentLimitConfig = new CurrentLimitsConfigs();
    private double m_targetAngle;
    private boolean m_isDeployed = false;
    private boolean m_isShaking = false;
    private final PositionVoltage m_deployPositionControl = new PositionVoltage(0);
    private final VoltageOut m_rollerVoltageControl = new VoltageOut(0);

    public Intake() {
        m_deployCurrentLimitConfig.withSupplyCurrentLimit(DEPLOY_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(DEPLOY_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true);
        m_rollerCurrentLimitConfig.withSupplyCurrentLimit(ROLLER_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(ROLLER_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true);
        m_deployMotorConfig.CurrentLimits = m_deployCurrentLimitConfig;
        m_rollerMotorConfig.CurrentLimits = m_rollerCurrentLimitConfig;

        m_deployMotorConfig.Slot0.kP = INTAKE_kP;
        m_deployMotorConfig.Slot0.kI = INTAKE_kI;
        m_deployMotorConfig.Slot0.kD = INTAKE_kD;

        m_rollerMotor.getConfigurator().apply(m_rollerMotorConfig);
        m_deployMotor.getConfigurator().apply(m_deployMotorConfig);
        m_rollerMotor.setNeutralMode(NeutralModeValue.Coast);
        m_deployMotor.setNeutralMode(NeutralModeValue.Coast);
        m_deployMotor.setPosition(0);
        m_rollerMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    }

    public double getMotorAngle() {
        return m_deployMotor.getPosition().getValueAsDouble();
    }

    public double getTargetAngle() {
        return m_targetAngle;
    }

    public double getRollerRPM() {
        return m_rollerMotor.getVelocity().getValueAsDouble() * 60;
    }

    public boolean isAtTarget() {
        return getMotorAngle() >= getTargetAngle() - ALLOWED_TARGET_ERROR
                && getMotorAngle() <= getTargetAngle() + ALLOWED_TARGET_ERROR;
    }

    private void moveToIntake(double targetPosition) {
        m_deployMotor.setControl(m_deployPositionControl.withPosition(targetPosition));
    }

    private void spinRoller(double voltage) {
        m_rollerVoltageControl.EnableFOC = true;
        m_rollerMotor.setControl(m_rollerVoltageControl.withOutput(voltage));
    }

    public void setIntakePID(double new_P, double new_I, double new_D) {
        m_deployMotorConfig.Slot0.kP = new_P;
        m_deployMotorConfig.Slot0.kI = new_I;
        m_deployMotorConfig.Slot0.kD = new_D;

        m_deployMotor.getConfigurator().apply(m_deployMotorConfig);
    }

    @Override
    public void periodic() {
        if (!m_isShaking) {
            if (m_isDeployed) {
                m_targetAngle = DEPLOYED_POSITION;
            } else {
                m_targetAngle = 0;
            }
        }
        spinRoller(m_rollerMotorVoltage);
        moveToIntake(m_targetAngle);
    }

    public Command runIntake(double speed) {
        return run(() -> {
            m_rollerMotorVoltage = speed;
        });
    }

    public Command moveIntakeIntermediate(double targetAngle, double speed) {
        return run(() -> {
            m_isShaking = true;
            m_targetAngle = targetAngle;
            m_rollerMotorVoltage = speed;
        });
    }

    public Command toggleIntake() {
        return runOnce(() -> {
            m_isDeployed = !m_isDeployed;
        });
    }

    public Command deployAndRunIntake(double speed) {
        return run(() -> {
            m_isDeployed = true;
            m_rollerMotorVoltage = speed;
        });
    }

    public Command retractIntake() {
        return run(() -> {
            m_isDeployed = false;
            m_rollerMotorVoltage = 0;
        });
    }

    public Command defaultCommand() {
        return run(() -> {
            m_isShaking = false;
            m_rollerMotorVoltage = 0;
        });
    }
}
