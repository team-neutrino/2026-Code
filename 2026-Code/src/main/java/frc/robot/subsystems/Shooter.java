// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.util.Constants.ShooterConstants.*;
import static frc.robot.util.Subsystems.hubState;
import static frc.robot.util.Subsystems.shooterArbiter;

import frc.robot.util.Constants.RioConstants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import static frc.robot.util.Constants.GlobalConstants.RED_ALLIANCE;
import static frc.robot.util.Subsystems.swerve; // this import is actually needed

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final CANBus m_CANbus = RioConstants.RIO_BUS;

  private TalonFX m_shooterMotor = new TalonFX(SHOOTER_ID, m_CANbus);
  private TalonFX m_shooterFollowerMotor = new TalonFX(SHOOTER_FOLLOWER_ID, m_CANbus);
  private TalonFX m_hoodMotor = new TalonFX(HOOD_ID, m_CANbus);

  private TalonFXConfiguration m_shooterMotorConfig = new TalonFXConfiguration();
  private TalonFXConfiguration m_hoodMotorConfig = new TalonFXConfiguration();
  private final CurrentLimitsConfigs m_currentLimitConfig = new CurrentLimitsConfigs();

  private double m_targetAngle = START_POSITION;

  private double m_targetShooterRpm = DEFAULT_SHOOTING_SPEED;

  private double m_tuningDistance = 5;

  /**
   * Creates a new Shooter.
   * 
   * @return A new shooter. What else would it give you
   */
  public Shooter() {
    m_currentLimitConfig.withSupplyCurrentLimit(CURRENT_LIMIT)
        .withSupplyCurrentLimitEnable(true)
        .withStatorCurrentLimit(CURRENT_LIMIT)
        .withStatorCurrentLimitEnable(true);
    m_shooterMotorConfig.CurrentLimits = m_currentLimitConfig;

    m_shooterMotorConfig.Slot0.kP = SHOOTING_KP;
    m_shooterMotorConfig.Slot0.kI = SHOOTING_KI;
    m_shooterMotorConfig.Slot0.kD = SHOOTING_KD;

    m_hoodMotorConfig.Slot0.kP = HOOD_KP;
    m_hoodMotorConfig.Slot0.kI = HOOD_KI;
    m_hoodMotorConfig.Slot0.kD = HOOD_KD;

    m_shooterMotor.getConfigurator().apply(m_shooterMotorConfig);
    m_shooterFollowerMotor.getConfigurator().apply(m_shooterMotorConfig);
    m_hoodMotor.getConfigurator().apply(m_hoodMotorConfig);
    m_shooterMotor.setNeutralMode(NeutralModeValue.Brake);
    m_shooterFollowerMotor.setNeutralMode(NeutralModeValue.Brake);
    m_hoodMotor.setNeutralMode(NeutralModeValue.Brake);

    Follower followRequest = new Follower(SHOOTER_ID, MotorAlignmentValue.Opposed);
    m_shooterFollowerMotor.setControl(followRequest);

    m_hoodMotor.setPosition(0.0);
  }

  /**
   * Sets a new PID controller for the shooter motor. Should only be used for
   * tuning the PID controller.
   * 
   * @param new_P The new P value for the shooter motor.
   * @param new_I The new I value for the shooter motor.
   * @param new_D The new D value for the shooter motor.
   */

  public void setShooterPID(double new_P, double new_I, double new_D) {
    m_shooterMotorConfig.Slot0.kP = new_P;
    m_shooterMotorConfig.Slot0.kI = new_I;
    m_shooterMotorConfig.Slot0.kD = new_D;

    m_shooterMotor.getConfigurator().apply(m_shooterMotorConfig);
  }

  /**
   * Sets a new PID controller for the hood motor. Should only be used for tuning
   * the PID controller.
   * 
   * @param new_P The new P value for the hood motor.
   * @param new_I The new I value for the hood motor.
   * @param new_D The new D value for the hood motor.
   */

  public void setHoodPID(double new_P, double new_I, double new_D) {
    m_hoodMotorConfig.Slot0.kP = new_P;
    m_hoodMotorConfig.Slot0.kI = new_I;
    m_hoodMotorConfig.Slot0.kD = new_D;

    m_hoodMotor.getConfigurator().apply(m_hoodMotorConfig);
  }

  /**
   * Gets the current position of the hood motor.
   * 
   * @return The current position of the hood motor as a double.
   */
  public double getHoodAngle() {
    return m_hoodMotor.getPosition().getValueAsDouble();
  }

  /**
   * Gets the current RPM of the shooter motor.
   * 
   * @return The current RPM of the shooter motor as a double.
   */
  public double getShooterRPM() {
    return m_shooterMotor.getVelocity().getValueAsDouble() * 60.0;
  }

  /**
   * Sets a new tuning distance for the tuning button on the controller to input
   * into the interpolation table.
   * 
   * @param newDistance The new distance for the tuning button.
   */
  public void setTuningDistance(double newDistance) {
    m_tuningDistance = newDistance;
  }

  /**
   * Gets the current tuning distance for the tuning button.
   * 
   * @return The current tuning distance for the tuning button.
   */
  public double getTuningDistance() {
    return m_tuningDistance;
  }

  /**
   * Gets the target position for the hood motor.
   * 
   * @return The target position for the hood motor.
   */
  public double getTargetPosition() {
    return m_targetAngle;
  }

  /**
   * Gets the target RPM for the hood motor.
   * 
   * @return The target RPM for the hood motor.
   */
  public double getTargetRPM() {
    return m_targetShooterRpm;
  }

  /**
   * Returns whether or not we are at the target position.
   * 
   * @return True if the shooter is at the target position, False if it is not.
   */
  public boolean atTargetPosition() {
    return Math.abs(getHoodAngle() - m_targetAngle) <= ALLOWED_ERROR;
  }

  /**
   * Returns whether or not we are at the target RPM.
   * 
   * @return True if the shooter is at the target RPM, false if it is not.
   */
  public boolean atTargetRPM() {
    return Math.abs(getShooterRPM() - m_targetShooterRpm) <= ALLOWED_RPM_ERROR;
  }

  /**
   * Returns whether or not we are ready to score.
   * 
   * @return True if the shooter is at the target position, False if it is not.
   */
  public boolean readyToScore() {
    return atTargetPosition();
  }

  /**
   * Sets the target that the hood motor should try to rotate to.
   * 
   * @param target The number of rotations the hood motor should make.
   */
  public void setTarget(double target) {
    m_targetAngle = target;
  }

  /**
   * Control the hood motor to reach a certain amount of rotations as defined by
   * m_TargetAngle.
   */
  public void controlHoodMotor() {
    PositionVoltage positionControl = new PositionVoltage(m_targetAngle);
    m_hoodMotor.setControl(positionControl);
  }

  /**
   * Control the shooter motor to target a certain RPM as decided by
   * m_targetShooterRpm.
   */
  public void controlShooterMotor() {
    VelocityVoltage velocityControl = new VelocityVoltage(m_targetShooterRpm / 60);
    m_shooterMotor.setControl(velocityControl);
  }

  @Override
  public void periodic() {
    controlShooterMotor();
    controlHoodMotor();

    shooterArbiter.setCondition(shooterConditions.SHOOTER_SPEED_CORRECT, atTargetRPM());
    shooterArbiter.setCondition(shooterConditions.HOOD_ANGLE_CORRECT, atTargetPosition());
    if (hubState.hasValidGameData()) {
      if (RED_ALLIANCE.get()) {
        shooterArbiter.setCondition(shooterConditions.HUB_ACTIVE,
            hubState.isRedHubActive());
      } else {
        shooterArbiter.setCondition(shooterConditions.HUB_ACTIVE,
            hubState.isBlueHubActive());
      }
    }
  }

  /**
   * A command to set the target shooting angle to a certain target.
   * 
   * @param target The target shooting angle in rotations.
   * @return A command to set the target shooting angle to a certain target.
   */
  public Command shootingAngle(double target) {
    return run(() -> {
      m_targetAngle = target;
    });
  }

  /**
   * A command to set the target shooting speed to a certain target.
   * 
   * @param speed The target shooting speed in rotations per minute.
   * @return A command to set the target shooting speed to a certain target.
   */
  public Command shootingSpeed(double speed) {
    return run(() -> {
      m_targetShooterRpm = speed;
    });
  }

  /**
   * A command to put the robot into shuttle mode.
   */
  public Command shuttle() {
    return run(() -> {
      m_targetAngle = SHUTTLE_ANGLE;
      m_targetShooterRpm = SHUTTLE_SHOOTING_SPEED;
    });
  }

  public Command defaultCommand() {
    return run(() -> {
      m_targetShooterRpm = SHOOTER_SPEED_ZONES.floorEntry(m_tuningDistance).getValue();
      m_targetAngle = INTERPOLATION_HOOD.get(m_tuningDistance);
      // m_targetShooterRpm =
      // SHOOTER_SPEED_ZONES.floorEntry(swerve.getDistanceFromHub()).getValue();
      // m_targetAngle = INTERPOLATION_HOOD.get(swerve.getDistanceFromHub());
    });
  }
}
