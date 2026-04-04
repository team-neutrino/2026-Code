// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.util.Constants.ShooterConstants.*;
import static frc.robot.util.Subsystems.shooterArbiter;

import frc.robot.util.Constants.RioConstants;
import frc.robot.util.Constants.ShooterConstants.shooterConditions;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import static frc.robot.util.Subsystems.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private final CANBus m_CANbus = RioConstants.RIO_BUS;

    private TalonFX m_shooterMotor = new TalonFX(SHOOTER_ID, m_CANbus);
    private TalonFX m_shooterFollowerMotor = new TalonFX(SHOOTER_FOLLOWER_ID, m_CANbus);
    private TalonFX m_hoodMotor = new TalonFX(HOOD_ID, m_CANbus);

    private TalonFXConfiguration m_shooterMotorConfig = new TalonFXConfiguration();
    private TalonFXConfiguration m_hoodMotorConfig = new TalonFXConfiguration();
    private final CurrentLimitsConfigs m_shooterCurrentLimitConfig = new CurrentLimitsConfigs();
    private final CurrentLimitsConfigs m_hoodCurrentLimitConfig = new CurrentLimitsConfigs();

    private double m_targetAngle = START_POSITION;

    private double m_targetShooterRpm = DEFAULT_SHOOTING_SPEED;

    private double m_tuningDistance = 5;

    private boolean m_resetting_hood = false;

    private double m_filteredSpeed;

    private final PositionVoltage m_hoodPositionControl = new PositionVoltage(0);
    private final VelocityVoltage m_shooterVelocityControl = new VelocityVoltage(0);

    public double m_tuningAngle;

    public double m_tuningSpeed;

    /**
     * Creates a new Shooter.
     */
    public Shooter() {
        m_shooterCurrentLimitConfig.withSupplyCurrentLimit(SHOOTER_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(SHOOTER_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true);
        m_shooterMotorConfig.CurrentLimits = m_shooterCurrentLimitConfig;

        m_hoodCurrentLimitConfig.withSupplyCurrentLimit(HOOD_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(HOOD_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true);
        m_hoodMotorConfig.CurrentLimits = m_hoodCurrentLimitConfig;

        m_shooterMotorConfig.Slot0.kP = SHOOTING_KP;
        m_shooterMotorConfig.Slot0.kI = SHOOTING_KI;
        m_shooterMotorConfig.Slot0.kD = SHOOTING_KD;
        m_shooterMotorConfig.Slot0.kV = SHOOTING_KV;

        m_hoodMotorConfig.Slot0.kP = HOOD_KP;
        m_hoodMotorConfig.Slot0.kI = HOOD_KI;
        m_hoodMotorConfig.Slot0.kD = HOOD_KD;

        m_shooterMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        m_hoodMotorConfig.Feedback.SensorToMechanismRatio = HOOD_GEAR_RATIO;

        m_shooterMotor.getConfigurator().apply(m_shooterMotorConfig);
        m_shooterFollowerMotor.getConfigurator().apply(m_shooterMotorConfig);
        m_hoodMotor.getConfigurator().apply(m_hoodMotorConfig);
        m_shooterMotor.setNeutralMode(NeutralModeValue.Brake);
        m_shooterFollowerMotor.setNeutralMode(NeutralModeValue.Brake);
        m_hoodMotor.setNeutralMode(NeutralModeValue.Coast);

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
        return m_hoodMotor.getPosition().getValueAsDouble() * 360;
    }

    /**
     * Gets the current RPM of the shooter motor.
     * 
     * @return The current RPM of the shooter motor as a double.
     */
    public double getShooterRPM() {
        return m_filteredSpeed;
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
        return Math.abs(getHoodAngle() - m_targetAngle) <= HOOD_ALLOWED_ERROR;
    }

    /**
     * Returns whether or not we are at the target RPM.
     * 
     * @return True if the shooter is at the target RPM, false if it is not.
     */
    public boolean atTargetRPM() {
        return Math.abs(getShooterRPM() - m_targetShooterRpm) <= RPM_ALLOWED_ERROR;
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
        if (m_resetting_hood) {
            m_hoodMotor.setVoltage(-1.0);
            return;
        }

        m_hoodMotor.setControl(m_hoodPositionControl.withPosition(getSafeAngle(m_targetAngle) / 360));
    }

    public double getHoodCurrent() {
        return Math.abs(m_hoodMotor.getTorqueCurrent().getValueAsDouble());
    }

    /**
     * Control the shooter motor to target a certain RPM as decided by
     * m_targetShooterRpm.
     */
    public void controlShooterMotor() {
        m_shooterMotor.setControl(m_shooterVelocityControl.withVelocity(m_targetShooterRpm / 60));
    }

    /**
     * Find a safe angle for the hood to travel to.
     * 
     * @param originalAngle The angle to set the hood to.
     * @return A safe angle for the hood to travel to.
     */
    public double getSafeAngle(double originalAngle) {
        return Math.min(originalAngle, MAX_SAFE_HOOD_ANGLE);
    }

    @Override
    public void periodic() {
        m_filteredSpeed = SHOOTER_RPM_NOISE * m_filteredSpeed
                + (1 - SHOOTER_RPM_NOISE) * (m_shooterMotor.getVelocity().getValueAsDouble()
                        * 60.0);

        shooterArbiter.setCondition(shooterConditions.SHOOTER_SPEED_CORRECT, atTargetRPM());
        shooterArbiter.setCondition(shooterConditions.HOOD_ANGLE_CORRECT, atTargetPosition());

        controlHoodMotor();
        controlShooterMotor();
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

    public Command shuttle() {
        return run(() -> {
            m_targetAngle = MAX_SAFE_HOOD_ANGLE;
            m_targetShooterRpm = SHUTTLE_SPEED_INTERPOLATION.get(swerve.getFromHubToTurret());
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

    public Command runShooterAndHood(double speed, double angle) {
        return run(() -> {
            m_targetShooterRpm = speed;
            m_targetAngle = angle;
        });
    }

    public Command resetHood() {
        return new FunctionalCommand(
                () -> {
                    m_resetting_hood = true;
                }, // set motor to go backwards on command start. please do make sure this is
                   // spinning the correct direction on the real robot or bad things will happen
                () -> {
                    controlShooterMotor(); // keep shooter running
                },
                interrupt -> {
                    m_hoodMotor.setPosition(0);
                    m_resetting_hood = false;
                },
                // set motor position to 0 when command ends
                () -> (Math.abs(m_hoodMotor.getTorqueCurrent().getValueAsDouble()) > CURRENT_SPIKE), // end command when
                                                                                                     // current
                                                                                                     // spike
                this // require shooter subsystem
        );
    }

    public Command defaultCommand() {
        return run(() -> {
            double hubDistance = swerve.getFromHubToTurret();
            if (swerve.inNeutralOrOpposingZone()) {
                m_targetAngle = HOOD_INTERPOLATION.get(hubDistance);
                m_targetShooterRpm = DEFAULT_SHOOTING_SPEED;
                return;
            }
            m_targetAngle = HOOD_INTERPOLATION.get(hubDistance);
            m_targetShooterRpm = SPEED_INTERPOLATION.get(hubDistance);

            // Manually tuning hood and speed
            // m_targetShooterRpm = m_tuningSpeed;
            // m_targetAngle = m_tuningAngle;
            // shooterArbiter.setCondition(shooterConditions.HUB_ACTIVE, false);
        });
    }

    public Command programmedShot() {
        return run(() -> {
            m_targetAngle = MAX_SAFE_HOOD_ANGLE;
            m_targetShooterRpm = PROGRAMMED_SHOT_SPEED;
        });
    }
}
