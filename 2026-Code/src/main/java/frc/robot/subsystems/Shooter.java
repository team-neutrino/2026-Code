// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.util.Constants.ShooterConstants.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final CANBus m_CANbus = new CANBus("rio");
  // motors
  private TalonFX m_motor1 = new TalonFX(16, m_CANbus);
  private TalonFX m_motor2 = new TalonFX(17, m_CANbus);
  private TalonFX m_hoodMotor = new TalonFX(19, m_CANbus);
  // configurations
  private TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();
  private TalonFXConfiguration m_hoodMotorConfig = new TalonFXConfiguration();
  private final CurrentLimitsConfigs m_currentLimitConfig = new CurrentLimitsConfigs();
  // sentenced to execution
  private double m_motorVoltage;

  public Shooter() {
    m_currentLimitConfig.withSupplyCurrentLimit(CURRENT_LIMIT)
        .withSupplyCurrentLimitEnable(true)
        .withStatorCurrentLimit(CURRENT_LIMIT)
        .withStatorCurrentLimitEnable(true);
    m_motorConfig.CurrentLimits = m_currentLimitConfig;

    m_motorConfig.Slot0.kP = SHOOTING_KP;
    m_motorConfig.Slot0.kI = SHOOTING_KI;
    m_motorConfig.Slot0.kD = SHOOTING_KD;

    m_hoodMotorConfig.Slot0.kP = HOOD_KP;
    m_hoodMotorConfig.Slot0.kI = HOOD_KI;
    m_hoodMotorConfig.Slot0.kD = HOOD_KD;

    m_motor1.getConfigurator().apply(m_motorConfig);
    m_motor2.getConfigurator().apply(m_motorConfig);
    m_hoodMotor.getConfigurator().apply(m_hoodMotorConfig);
    m_motor1.setNeutralMode(NeutralModeValue.Coast);
    m_motor2.setNeutralMode(NeutralModeValue.Coast);
    m_hoodMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  public double getVelocity() {
    return m_motor1.getVelocity().getValueAsDouble();
  }

  @Override
  public void periodic() {
    m_motor1.setVoltage(m_motorVoltage);
    m_motor2.setVoltage(-m_motorVoltage);
  }

  public Command runShooter(double speed) {
    return run(() -> {
      m_motorVoltage = speed;
    });
  }

  public Command defaultCommand() {
    return run(() -> {
      m_motorVoltage = 0;
    });
  }
}
