// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.util.Constants.ShooterConstants.*;


public class Shooter extends SubsystemBase {
  private final CANBus m_CANbus = new CANBus("rio");
  private TalonFX m_motor1 = new TalonFX(11, m_CANbus);
  private TalonFX m_motor2 = new TalonFX(12, m_CANbus);
  private TalonFX m_indexer = new TalonFX(13, m_CANbus);
  private TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();
  private final CurrentLimitsConfigs m_currentLimitConfig = new CurrentLimitsConfigs();
  private double m_motorVoltage;
  private double m_indexerVoltage;

  public Shooter() {
    m_currentLimitConfig.withSupplyCurrentLimit(CURRENT_LIMIT)
    .withSupplyCurrentLimitEnable(true)
    .withStatorCurrentLimit(CURRENT_LIMIT)
    .withStatorCurrentLimitEnable(true);
    m_motorConfig.CurrentLimits = m_currentLimitConfig;

    m_motor1.getConfigurator().apply(m_motorConfig);
    m_motor2.getConfigurator().apply(m_motorConfig);
    m_indexer.getConfigurator().apply(m_motorConfig);
    m_motor1.setNeutralMode(NeutralModeValue.Coast);
    m_motor2.setNeutralMode(NeutralModeValue.Coast);
    m_indexer.setNeutralMode(NeutralModeValue.Coast);
  }

  public double getVelocity() {
    return m_motor1.getVelocity().getValueAsDouble();
  }

  public Command runShooter() {
    return run(() -> {
      m_motorVoltage = SHOOTING_VOLTAGE;
    });
  }

  public Command runIndexer() {
    return run(() -> {
      m_indexerVoltage = INDEXING_VOLTAGE;
    });
  }

  public Command defaultCommand() {
    return run(() -> {
      m_motorVoltage = 0;
      m_indexerVoltage = 0;
    });
  }

  @Override
  public void periodic() {
    m_motor1.setVoltage(m_motorVoltage);
    m_motor2.setVoltage(-m_motorVoltage);
    m_indexer.setVoltage(m_indexerVoltage);
    System.out.println(getVelocity());
  }
}
