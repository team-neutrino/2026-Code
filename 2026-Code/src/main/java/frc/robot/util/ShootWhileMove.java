// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.util.Subsystems.shooter;
import static frc.robot.util.Subsystems.swerve;

public class ShootWhileMove extends SubsystemBase {
  double m_desiredVelocity;
  double m_xPos;
  double m_yPos;
  double m_xVel;
  double m_yVel;
  double m_xAcc;
  double m_yAcc;
  double m_yaw;
  double m_desiredVelocityComponents[];

  public ShootWhileMove() {
  }

  private double getDesiredVelocity() {
    return shooter.getInterpolatedRPM();
  }

  private void getPosition() {
    m_xPos = swerve.getCurrentPose().getX();
    m_yPos = swerve.getCurrentPose().getY();
  }

  private void getVelocity() {
    m_xVel = swerve.getChassisSpeeds().vxMetersPerSecond;
    m_yVel = swerve.getChassisSpeeds().vyMetersPerSecond;
  }

  private void getAcceleration() {
    m_xAcc = swerve.getAccelerationX();
    m_yAcc = swerve.getAccelerationY();
  }

  private double[] getDesiredVelocityComponents() {
    m_yaw = swerve.getCurrentPose().getRotation().getDegrees();
    m_desiredVelocityComponents[0] = Math.cos(m_yaw) * m_xVel;
    m_desiredVelocityComponents[1] = Math.sin(m_yaw) * m_yVel;
    return m_desiredVelocityComponents;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
