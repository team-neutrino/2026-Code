// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.GlobalConstants;
import frc.robot.util.Subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.util.Constants.FieldMeasurementConstants.*;
import static frc.robot.util.Constants.RioConstants.*;
import static frc.robot.util.Constants.TurretConstants.*;

public class Turret extends SubsystemBase {

  private TalonFX m_motor = new TalonFX(MOTOR_ID, RIO_BUS);
  private double m_targetAngle = STARTUP_ANGLE;
  private double m_previousAngle = STARTUP_ANGLE;
  private double m_totalWrap = STARTUP_ANGLE;
  private double m_turret_angle = 0;
  private TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();
  private final CurrentLimitsConfigs m_currentLimitConfig = new CurrentLimitsConfigs();
  private final MotionMagicVoltage m_motionMagicRequest = new MotionMagicVoltage(STARTUP_ANGLE).withSlot(0);
  private final CANcoder m_encoder = new CANcoder(ENCODER_ID, RIO_BUS);
  private int printCount = 0;

  private final StatusSignal<Boolean> f_fusedSensorOutOfSync = m_motor.getFault_FusedSensorOutOfSync(false);
  private final StatusSignal<Boolean> sf_fusedSensorOutOfSync = m_motor.getStickyFault_FusedSensorOutOfSync(false);
  private final StatusSignal<Boolean> f_remoteSensorInvalid = m_motor.getFault_RemoteSensorDataInvalid(false);
  private final StatusSignal<Boolean> sf_remoteSensorInvalid = m_motor.getStickyFault_RemoteSensorDataInvalid(false);

  private final StatusSignal<Angle> m_position = m_motor.getPosition(false);
  private final StatusSignal<AngularVelocity> m_velocity = m_motor.getVelocity(false);
  private final StatusSignal<Angle> m_encoderPosition = m_encoder.getPosition(false);
  private final StatusSignal<AngularVelocity> m_encoderVelocity = m_encoder.getVelocity(false);
  private final StatusSignal<Angle> m_rotorPosition = m_motor.getRotorPosition(false);

  NetworkTableInstance nt = NetworkTableInstance.getDefault();
  private final NetworkTable driveStateTable = nt.getTable("Turret");
  private final StructPublisher<Pose2d> turretPosePub = driveStateTable.getStructTopic("TurretPose", Pose2d.struct)
      .publish();

  public Turret() {
    m_currentLimitConfig.withSupplyCurrentLimit(CURRENT_LIMIT)
        .withSupplyCurrentLimitEnable(true)
        .withStatorCurrentLimit(CURRENT_LIMIT)
        .withStatorCurrentLimitEnable(true);
    m_motorConfig.CurrentLimits = m_currentLimitConfig;

    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(DISCONTINUITY_POINT));
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    encoderConfig.MagnetSensor.withMagnetOffset(Rotations.of(ENCODER_MAGNET_OFFSET));
    m_encoder.getConfigurator().apply(encoderConfig);

    m_motorConfig.Feedback.FeedbackRemoteSensorID = m_encoder.getDeviceID();
    m_motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    m_motorConfig.Feedback.SensorToMechanismRatio = SENSOR_TO_MECHANISM_RATIO;
    m_motorConfig.Feedback.RotorToSensorRatio = ROTOR_TO_SENSOR_RATIO;

    var slot0Configs = m_motorConfig.Slot0;
    slot0Configs.kS = STATIC_FF; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = VELOCITY_FF; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = ACCELERATION_FF; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = TURRET_P; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = TURRET_I; // no output for integrated error
    slot0Configs.kD = TURRET_D; // A velocity error of 1 rps results in 0.1 V output

    var motionMagicConfigs = m_motorConfig.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = TARGET_CRUISE_VELOCITY;
    motionMagicConfigs.MotionMagicAcceleration = TARGET_ACCELERATION;
    motionMagicConfigs.MotionMagicJerk = TARGET_JERK;

    m_motor.getConfigurator().apply(m_motorConfig);

    m_motor.setNeutralMode(NeutralModeValue.Brake);
    m_motor.setPosition(STARTUP_ANGLE);
  }

  public double getCurrentAngle() {
    return m_motor.getPosition().getValueAsDouble();
  }

  public double getFieldRelativeAngle() {
    return m_motor.getPosition().getValueAsDouble();
  }

  public double getCurrentVoltage() {
    return m_motor.getMotorVoltage().getValueAsDouble();
  }

  private void adjustTurret(double targetAngle) {
    double robotAngularVelocity = Subsystems.swerve.getPigeon2().getAngularVelocityZDevice()
        .getValueAsDouble();
    m_motor
        .setControl(
            m_motionMagicRequest.withPosition(targetAngle).withFeedForward(-robotAngularVelocity * TURRET_FF));
  }

  private double getAdjustedTargetAngle() {
    Rotation2d turret_rotation = new Rotation2d(Math.toRadians(m_turret_angle));
    Rotation2d turrent_angle_global = turret_rotation.plus(Subsystems.swerve.getCurrentPose().getRotation());

    double turrent_angle_global_degrees = MathUtil.inputModulus(
        turrent_angle_global.getDegrees(),
        -180.0,
        180.0);
    double angleDiff = turrent_angle_global_degrees - calculateFieldRelativeTargetAngle();
    double closeTarget;
    double farTarget;
    if (Math.abs(angleDiff) < Math
        .abs(GlobalConstants.RED_ALLIANCE.get() ? (angleDiff <= 0 ? angleDiff + 360 : angleDiff - 360)
            : (angleDiff >= 0 ? angleDiff + 360 : angleDiff - 360))) {
      closeTarget = angleDiff;
      farTarget = GlobalConstants.RED_ALLIANCE.get() ? (angleDiff <= 0 ? angleDiff + 360 : angleDiff - 360)
          : (angleDiff >= 0 ? angleDiff + 360 : angleDiff - 360);
    } else {
      closeTarget = GlobalConstants.RED_ALLIANCE.get() ? (angleDiff <= 0 ? angleDiff + 360 : angleDiff - 360)
          : (angleDiff >= 0 ? angleDiff + 360 : angleDiff - 360);
      farTarget = angleDiff;
    }
    System.out.println(
        "Turret Angle: " + turrent_angle_global_degrees + " Target Angle: " + calculateFieldRelativeTargetAngle()
            + " AngleDiff: " + angleDiff + " closeTarget: " + closeTarget + " totalWindup: " + m_totalWrap);
    if (m_totalWrap - closeTarget >= MAX_WINDUP) {
      return m_totalWrap - closeTarget - 360;
    } else if (m_totalWrap - closeTarget <= MIN_WINDUP) {
      return m_totalWrap - closeTarget + 360;
    }
    return m_totalWrap - closeTarget;

  }

  public boolean isAtTarget() {
    double currentAngle = getCurrentAngle();
    return currentAngle <= m_targetAngle + ALLOWED_ERROR && currentAngle >= m_targetAngle - ALLOWED_ERROR;
  }

  @Override
  public void periodic() {
    if (++printCount >= 10) {
      printCount = 0;
      BaseStatusSignal.refreshAll(
          f_fusedSensorOutOfSync,
          sf_fusedSensorOutOfSync,
          f_remoteSensorInvalid,
          sf_remoteSensorInvalid,
          m_position, m_velocity,
          m_encoderPosition, m_encoderVelocity);
    }

    boolean anyFault = sf_fusedSensorOutOfSync.getValue() || sf_remoteSensorInvalid.getValue();
    if (anyFault) {
      System.out.println("A fault has occurred:");
      /*
       * If we're live, indicate live, otherwise if we're sticky indicate sticky,
       * otherwise do nothing
       */
      if (f_fusedSensorOutOfSync.getValue()) {
        System.out.println("Fused sensor out of sync live-faulted");
      } else if (sf_fusedSensorOutOfSync.getValue()) {
        System.out.println("Fused sensor out of sync sticky-faulted");
      }
      /*
       * If we're live, indicate live, otherwise if we're sticky indicate sticky,
       * otherwise do nothing
       */
      if (f_remoteSensorInvalid.getValue()) {
        System.out.println("Missing remote sensor live-faulted");
      } else if (sf_remoteSensorInvalid.getValue()) {
        System.out.println("Missing remote sensor sticky-faulted");
      }
    }

    final long now = NetworkTablesJNI.now();
    if (GlobalConstants.RED_ALLIANCE.isPresent()) {
      updateWrap();
      getAdjustedTargetAngle();
      adjustTurret(getAdjustedTargetAngle());
      m_turret_angle = simulateTurretMovement(getAdjustedTargetAngle());
      turretPosePub.set(new Pose2d(Subsystems.swerve.getCurrentPose().getMeasureX().baseUnitMagnitude(),
          Subsystems.swerve.getCurrentPose().getMeasureY().baseUnitMagnitude(),
          new Rotation2d((m_turret_angle + Subsystems.swerve.getCurrentPose().getRotation().getDegrees())
              * Math.PI / 180)),
          now);
    }
  }

  private double simulateTurretMovement(double turret_target_turret_space) {
    double delta_turret = turret_target_turret_space - m_turret_angle;
    double dt = 0.02;
    double max_turret_rot_rate = 1;
    double max_turret_movement = delta_turret * max_turret_rot_rate * dt;
    double constrained_delta_turret;
    if (Math.abs(delta_turret) < Math.abs(max_turret_movement)) {
      constrained_delta_turret = delta_turret;
    } else {
      constrained_delta_turret = max_turret_movement;
    }

    return m_turret_angle + constrained_delta_turret;
  }

  private void updateWrap() {
    double current = m_turret_angle;
    double delta = current - m_previousAngle;

    m_totalWrap += delta;
    m_previousAngle = current;
  }

  private double calculateFieldRelativeTargetAngle() {
    Pose2d robotPose = Subsystems.swerve.getCurrentPose();
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

    return Math.toDegrees(Math.atan2(targetDistanceY, targetDistanceX));
  }

  private double calculateRobotRelativeTargetAngle() {
    return calculateFieldRelativeTargetAngle() - Subsystems.swerve.getCurrentPose().getRotation().getDegrees();
  }

  public Command defaultCommand() {
    return run(() -> {
      m_targetAngle = calculateRobotRelativeTargetAngle();
    });
  }
}
