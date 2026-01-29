// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.RioConstants;

import static frc.robot.util.Constants.KickerConstants.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static frc.robot.util.Subsystems2026.*;

public class Kicker extends SubsystemBase {

  private final CANBus m_CANbus = RioConstants.RIO_BUS;
  private TalonFX m_kickerMotor = new TalonFX(KICKER_MOTOR_ID, m_CANbus);
  private TalonFXConfiguration m_kickerMotorConfig = new TalonFXConfiguration();
  private final CurrentLimitsConfigs m_currentLimitConfig = new CurrentLimitsConfigs();
  private double m_kickerMotorVoltage;

  /** Creates a new Kicker. */
  public Kicker() {
    m_currentLimitConfig.withSupplyCurrentLimit(KICKER_CURRENT_LIMIT)
        .withSupplyCurrentLimitEnable(true)
        .withStatorCurrentLimit(KICKER_CURRENT_LIMIT)
        .withStatorCurrentLimitEnable(true);
    m_kickerMotorConfig.CurrentLimits = m_currentLimitConfig;

    m_kickerMotor.getConfigurator().apply(m_kickerMotorConfig);
    m_kickerMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  public Command runKicker() {
    return run(() -> {
      m_kickerMotorVoltage = KICKER_VOLTAGE;
    });
  }

  public Command defaultCommand() {
    return run(() -> {
      m_kickerMotorVoltage = 0;
    });
  }

  @Override
  public void periodic() {
    m_kickerMotor.setVoltage(m_kickerMotorVoltage);
    index.checkHopperCapacity(m_kickerMotorVoltage);
  }
}
