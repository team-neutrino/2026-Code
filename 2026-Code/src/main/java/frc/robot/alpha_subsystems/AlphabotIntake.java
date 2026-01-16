// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.alpha_subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.util.Constants.IntakeConstants.*;

public class AlphabotIntake extends SubsystemBase {
  private final CANBus m_CANbus = new CANBus("rio");
  private TalonFX m_motor = new TalonFX(13, m_CANbus);
  private TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();
  private final CurrentLimitsConfigs m_currentLimitConfig = new CurrentLimitsConfigs();
  private double m_motorVoltage;

  public AlphabotIntake() {
    m_currentLimitConfig.withSupplyCurrentLimit(CURRENT_LIMIT)
        .withSupplyCurrentLimitEnable(true)
        .withStatorCurrentLimit(CURRENT_LIMIT)
        .withStatorCurrentLimitEnable(true);
    m_motorConfig.CurrentLimits = m_currentLimitConfig;

    m_motor.getConfigurator().apply(m_motorConfig);
    m_motor.setNeutralMode(NeutralModeValue.Coast);
  }

  public double getVelocity() {
    return m_motor.getVelocity().getValueAsDouble();
  }

  public Command runIntake() {
    return run(() -> {
      m_motorVoltage = INTAKE_VOLTAGE;
    });
  }

  public Command runOuttake() {
    return run(() -> {
      m_motorVoltage = -INTAKE_VOLTAGE;
    });
  }

  public Command defaultCommand() {
    return run(() -> {
      m_motorVoltage = 0;
    });
  }

  @Override
  public void periodic() {
    m_motor.setVoltage(m_motorVoltage);
  }
}
