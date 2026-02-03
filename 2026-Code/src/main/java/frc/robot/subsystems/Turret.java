// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.AlphaSubsystem;
import frc.robot.util.Constants.GlobalConstants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static frc.robot.util.Constants.FieldMeasurementConstants.*;
import static frc.robot.util.Constants.RioConstants.*;
import static frc.robot.util.Constants.TurretConstants.*;

public class Turret extends SubsystemBase {

  private TalonFX m_motor = new TalonFX(MOTOR_ID, RIO_BUS);
  private double m_targetAngle = STARTUP_ANGLE;
  private TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();
  private final CurrentLimitsConfigs m_currentLimitConfig = new CurrentLimitsConfigs();
  private final MotionMagicVoltage m_motionMagicRequest = new MotionMagicVoltage(STARTUP_ANGLE).withSlot(0);
  private final PositionVoltage m_positionVoltageRequest = new PositionVoltage(STARTUP_ANGLE).withSlot(1);

  public Turret() {
    m_currentLimitConfig.withSupplyCurrentLimit(CURRENT_LIMIT)
        .withSupplyCurrentLimitEnable(true)
        .withStatorCurrentLimit(CURRENT_LIMIT)
        .withStatorCurrentLimitEnable(true);
    m_motorConfig.CurrentLimits = m_currentLimitConfig;

    m_motorConfig.Feedback.SensorToMechanismRatio = SENSOR_TO_MECHANISM_RATIO;

    var slot0Configs = m_motorConfig.Slot0;
    slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    var motionMagicConfigs = m_motorConfig.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 1; // Target cruise velocity of 1 rps
    motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    var slot1Configs = m_motorConfig.Slot1;
    slot1Configs.kP = TURRET_P;
    slot1Configs.kI = TURRET_I;
    slot1Configs.kD = TURRET_D;

    m_motor.getConfigurator().apply(slot0Configs);
    m_motor.setNeutralMode(NeutralModeValue.Brake);
    m_motor.setPosition(STARTUP_ANGLE);
  }

  public double getCurrentAngle() {
    return m_motor.getPosition().getValueAsDouble();
  }

  private void adjustTurret(double targetAngle) {
    double robotAngularVelocity = AlphaSubsystem.swerve.getPigeon2().getAngularVelocityZDevice()
        .getValueAsDouble();
    m_motor
        .setControl(m_motionMagicRequest.withPosition(targetAngle).withFeedForward(-robotAngularVelocity * TURRET_FF));
  }

  private double getClampedTargetAngle() {
    return MathUtil.clamp(m_targetAngle, 0, 360);
  }

  public boolean isAtTarget() {
    double currentAngle = getCurrentAngle();
    return currentAngle <= m_targetAngle + ALLOWED_ERROR && currentAngle >= m_targetAngle - ALLOWED_ERROR;
  }

  @Override
  public void periodic() {
    adjustTurret(getClampedTargetAngle());
  }

  private double calculateTargetAngle() {
    Pose2d robotPose = AlphaSubsystem.swerve.getCurrentPose();
    double robotX = robotPose.getMeasureX().baseUnitMagnitude();
    double robotY = robotPose.getMeasureY().baseUnitMagnitude();

    Pose2d hubPose = GlobalConstants.RED_ALLIANCE.get() ? RED_HUB : BLUE_HUB;
    Pose2d shuttlePose = GlobalConstants.RED_ALLIANCE.get()
        ? (robotY > MID_FIELD ? SHUTTLE_TARGET_TOP_RED : SHUTTLE_TARGET_BOTTOM_RED)
        : (robotY > MID_FIELD ? SHUTTLE_TARGET_TOP_BLUE : SHUTTLE_TARGET_BOTTOM_BLUE);

    boolean isInAllianceZone = (GlobalConstants.RED_ALLIANCE.get() && robotX >= ALLIANCE_ZONE_RED)
        || (!GlobalConstants.RED_ALLIANCE.get() && robotX <= ALLIANCE_ZONE_BLUE);

    Pose2d targetPose = isInAllianceZone ? hubPose : shuttlePose;
    double targetDistanceX = targetPose.getX() - robotX; // add turret offset from center
    double targetDistanceY = targetPose.getY() - robotY;

    return Math.toDegrees(Math.atan2(targetDistanceY, targetDistanceX)) + robotPose.getRotation().getDegrees();
  }

  public Command defaultCommand() {
    return run(() -> {
      m_targetAngle = calculateTargetAngle();
    });
  }
}
