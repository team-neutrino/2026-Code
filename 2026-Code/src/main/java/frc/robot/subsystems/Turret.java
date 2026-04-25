// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.GlobalConstants;
import frc.robot.util.Constants.ShooterConstants.shooterConditions;
import frc.robot.util.Subsystems;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.util.Constants.GlobalConstants.RED_ALLIANCE;
import static frc.robot.util.Constants.RioConstants.*;
import static frc.robot.util.Constants.TurretConstants.*;
import static frc.robot.util.Subsystems.shooterArbiter;

public class Turret extends SubsystemBase {

    private TalonFX m_motor = new TalonFX(MOTOR_ID, RIO_BUS);
    private double m_targetAngle = STARTUP_ANGLE;
    private double m_previousAngle = STARTUP_ANGLE;
    private double m_totalWrap = STARTUP_ANGLE;
    private double m_adjustedTargetAngle = STARTUP_ANGLE;
    private double m_previousFieldRelativeTargetAngle = 0.0;
    private TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();
    private final CurrentLimitsConfigs m_currentLimitConfig = new CurrentLimitsConfigs();
    private final CANcoder m_encoder = new CANcoder(ENCODER_ID, RIO_BUS);
    private final PositionVoltage m_turretPositionControl = new PositionVoltage(0);

    // Simulation
    private TurretSim m_turretSim;
    private double m_staticFF;

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
        m_motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        m_motorConfig.Feedback.SensorToMechanismRatio = SENSOR_TO_MECHANISM_RATIO;
        m_motorConfig.Feedback.RotorToSensorRatio = ROTOR_TO_SENSOR_RATIO;

        var slot0Configs = m_motorConfig.Slot0;
        slot0Configs.kS = 0.0;
        slot0Configs.kV = VELOCITY_FF;
        slot0Configs.kA = ACCELERATION_FF;
        slot0Configs.kP = TURRET_P;
        slot0Configs.kI = TURRET_I;
        slot0Configs.kD = TURRET_D;

        m_motor.getConfigurator().apply(m_motorConfig);

        m_motor.setNeutralMode(NeutralModeValue.Coast);
        m_motor.setPosition(STARTUP_ANGLE);
        m_encoder.setPosition(STARTUP_ANGLE);

        // Initialize simulation
        if (RobotBase.isSimulation()) {
            m_turretSim = new TurretSim(m_motor, m_encoder);
            m_turretSim.resetSimulation(STARTUP_ANGLE);
        }
    }

    public double calculateTurretAngleDifference(double currentAngle, double previousAngle) {
        if (Math.abs(previousAngle - currentAngle) > 180) {
            if (previousAngle < 0) {
                return currentAngle - (previousAngle + 360);
            } else {
                return (currentAngle + 360) - previousAngle;
            }
        } else {
            return currentAngle - previousAngle;
        }
    }

    public double staticFeedforward() {
        if (getCurrentAngle() > 90) {
            return m_staticFF = STATIC_FF;
        } else {
            return m_staticFF = -STATIC_FF;
        }
    }

    @Override
    public void periodic() {
        if (GlobalConstants.RED_ALLIANCE.isPresent()) {
            updateWrap();
            m_adjustedTargetAngle = getAdjustedTargetAngle();

            final double fieldRelativeTargetAngle = Subsystems.swerve.getFieldRelativeTargetAngle();
            final double translationRate = calculateTurretAngleDifference(fieldRelativeTargetAngle,
                    m_previousFieldRelativeTargetAngle) * 50;
            m_previousFieldRelativeTargetAngle = fieldRelativeTargetAngle;
            final double rotationRate = -Math.toDegrees(Subsystems.swerve.getChassisSpeeds().omegaRadiansPerSecond);

            adjustTurret(m_adjustedTargetAngle, translationRate, rotationRate);
        }
        shooterArbiter.setCondition(shooterConditions.TURRET_ANGLE_CORRECT, isAtTarget());
    }

    public double getCurrentAngle() {
        return m_motor.getPosition().getValueAsDouble() * 360;
    }

    public double getTargetAngle() {
        return m_targetAngle;
    }

    public double getAbsoluteEncoderValue() {
        return m_encoder.getAbsolutePosition().getValueAsDouble();
    }

    public double getCurrentVoltage() {
        return m_motor.getMotorVoltage().getValueAsDouble();
    }

    public double getAdjustedTargetAngle() {
        Rotation2d turret_rotation = new Rotation2d(Math.toRadians(getCurrentAngle()));
        Rotation2d turrent_angle_global = turret_rotation.plus(Subsystems.swerve.getCurrentPose().getRotation());

        double turrent_angle_global_degrees = MathUtil.inputModulus(
                turrent_angle_global.getDegrees(),
                -180.0,
                180.0);
        double angleDiff = turrent_angle_global_degrees
                - (m_targetAngle + Subsystems.swerve.getCurrentPose().getRotation().getDegrees());
        double closeTarget;
        if (Math.abs(angleDiff) < Math
                .abs(GlobalConstants.RED_ALLIANCE.get() ? (angleDiff <= 0 ? angleDiff + 360 : angleDiff - 360)
                        : (angleDiff >= 0 ? angleDiff + 360 : angleDiff - 360))) {
            closeTarget = angleDiff;
        } else {
            closeTarget = GlobalConstants.RED_ALLIANCE.get() ? (angleDiff <= 0 ? angleDiff + 360 : angleDiff - 360)
                    : (angleDiff >= 0 ? angleDiff + 360 : angleDiff - 360);
        }
        if (m_totalWrap - closeTarget >= MAX_WINDUP) {
            return m_totalWrap - closeTarget - 360;
        } else if (m_totalWrap - closeTarget <= MIN_WINDUP) {
            return m_totalWrap - closeTarget + 360;
        }
        return m_totalWrap - closeTarget;
    }

    public boolean isAtTarget() {
        double targetAngle = m_targetAngle;
        double currentAngle = getCurrentAngle();
        if (RED_ALLIANCE.isPresent()) {
            targetAngle = m_adjustedTargetAngle;
        }
        return Math.abs(currentAngle - targetAngle) < ALLOWED_ERROR;
    }

    private double voltageFeedForward(double translationRate, double rotationRate) {
        double feedforward = staticFeedforward();
        if (calculateTurretAngleDifference(getCurrentAngle(), m_targetAngle) < TRACKING_THRESHOLD) {
            feedforward += translationRate / 360.0 * TURRET_TRACKING_TRANSLATION_KV;
            feedforward += rotationRate / 360.0 * TURRET_TRACKING_ROTATION_KV;
        }
        return feedforward;
    }

    private void adjustTurret(double targetAngle, double translationRate, double rotationRate) {
        m_motor.setControl(
                m_turretPositionControl.withPosition(targetAngle / 360)
                        .withFeedForward(voltageFeedForward(translationRate, rotationRate)));
    }

    private void updateWrap() {
        double current = getCurrentAngle();
        double delta = current - m_previousAngle;

        m_totalWrap += delta;
        m_previousAngle = current;
    }

    private double calculateRobotRelativeTargetAngle() {
        return Subsystems.swerve.getFieldRelativeTargetAngle()
                - Subsystems.swerve.getCurrentPose().getRotation().getDegrees();
    }

    public Command defaultCommand() {
        return run(() -> {
            m_targetAngle = calculateRobotRelativeTargetAngle();
        });
    }

    public Command setTargetAngleCommand(double targetAngle) {
        return run(() -> {
            m_targetAngle = targetAngle;
        });
    }

    public void changePID(double p, double i, double d) {
        m_motorConfig.Slot0.kP = p;
        m_motorConfig.Slot0.kI = i;
        m_motorConfig.Slot0.kD = d;
        m_motor.getConfigurator().apply(m_motorConfig);
    }

    @Override
    public void simulationPeriodic() {
        if (m_turretSim != null) {
            m_turretSim.updateSimulation();
        }
    }
}
