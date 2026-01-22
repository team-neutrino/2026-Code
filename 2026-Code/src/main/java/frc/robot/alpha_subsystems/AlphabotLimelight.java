// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.alpha_subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;
import frc.robot.util.AlphaSubsystem;
import frc.robot.subsystems.Swerve;
import static frc.robot.util.Constants.AlphabotLimelightConstants.*;

//Uncommment everything with swerve in it when swerve is added

public class AlphabotLimelight extends SubsystemBase {
  LimelightHelpers m_limelightHelpers;
  double m_robotYaw;
  Swerve m_swerve;
  Rotation2d m_targetYaw;
  private double m_lastFrameBr = -2;
  private double m_lastFrameBl = -2;
  private double m_lastFrameShooter = -2;
  private boolean m_has_shooter_tag;
  private boolean m_has_br_tag;
  private boolean m_has_bl_tag;
  private boolean m_enabled = false;
  private long m_slow_count = 0;

  public AlphabotLimelight() {
    m_swerve = AlphaSubsystem.swerve;
    m_limelightHelpers = new LimelightHelpers();
    // fake pipeline number
    // LimelightHelpers.setPipelineIndex(LIMELIGHT_1, 1);
    LimelightHelpers.setLEDMode_ForceOff(AlphaLL_BL);
    LimelightHelpers.setCameraPose_RobotSpace(AlphaLL_BL,
        AlphaBL_FORWARD_OFFSET, // Forward offset (meters)
        AlphaBL_SIDE_OFFSET, // Side offset (meters) left is positive
        AlphaBL_HEIGHT_OFFSET, // Height offset (meters)
        AlphaBL_ROLL_OFFSET, // Roll (degrees)
        AlphaBL_PITCH_OFFSET, // Pitch (degrees)
        AlphaBL_YAW_OFFSET // Yaw (degrees)
    );
    LimelightHelpers.SetFiducialDownscalingOverride(AlphaLL_BL, 3);

    LimelightHelpers.setLEDMode_ForceOff(AlphaLL_BR);
    LimelightHelpers.setCameraPose_RobotSpace(AlphaLL_BR,
        AlphaBR_FORWARD_OFFSET, // Forward offset (meters)
        AlphaBR_SIDE_OFFSET, // Side offset (meters) left is positive
        AlphaBR_HEIGHT_OFFSET, // Height offset (meters)
        AlphaBR_ROLL_OFFSET, // Roll (degrees)
        AlphaBR_PITCH_OFFSET, // Pitch (degrees)
        AlphaBR_YAW_OFFSET // Yaw (degrees)
    );
    LimelightHelpers.SetFiducialDownscalingOverride(AlphaLL_BR, 3);

    LimelightHelpers.setLEDMode_ForceOff(AlphaLL_SHOOTER);
    LimelightHelpers.setCameraPose_RobotSpace(AlphaLL_SHOOTER,
        AlphaSHOOTER_FORWARD_OFFSET, // Forward offset (meters)
        AlphaSHOOTER_SIDE_OFFSET, // Side offset (meters) left is positive
        AlphaSHOOTER_HEIGHT_OFFSET, // Height offset (meters)
        AlphaSHOOTER_ROLL_OFFSET, // Roll (degrees)
        AlphaSHOOTER_PITCH_OFFSET, // Pitch (degrees)
        AlphaSHOOTER_YAW_OFFSET // Yaw (degrees)
    );

    LimelightHelpers.SetFiducialDownscalingOverride(AlphaLL_SHOOTER, 3);

    LimelightHelpers.SetIMUMode(AlphaLL_SHOOTER, 1);
    LimelightHelpers.SetIMUMode(AlphaLL_BL, 1);
    // use external IMU yaw submitted via setRobotOrientation() and configure the
    // LL4 internal IMU's fused yaw to match the submitted yaw value
    // 0 - Use external IMU yaw submitted via SetRobotOrientation() for MT2
    // localization. The internal IMU is ignored entirely.
    // 1 - Use external IMU yaw submitted via SetRobotOrientation(), and configure
    // the LL4 internal IMU's fused yaw to match the submitted yaw value.
    // 2 - Use internal IMU for MT2 localization.
  }

  /** True when the shooter camera currently sees a fiducial. */
  public boolean hasShooterTag() {
    return m_has_shooter_tag;
  }

  /** True when the back-right camera currently sees a fiducial. */
  public boolean hasBackRightTag() {
    return m_has_br_tag;
  }

  /** True when the back-left camera currently sees a fiducial. */
  public boolean hasBackLeftTag() {
    return m_has_bl_tag;
  }

  private boolean verifyLimelightValidity(PoseEstimate estimate, double frame) {
    return (estimate != null
        && estimate.tagCount != 0
    // && m_swerve.getState().Speeds.omegaRadiansPerSecond < 4 * Math.PI
    // && frame > m_lastFrameShooter
    ); // TODO fix this soon but it doesn't apply to this pull request
  }

  private void updateFrame(double frame, String name) {
    switch (name) {
      case AlphaLL_BR:
        m_lastFrameBr = frame;
        break;
      case AlphaLL_BL:
        m_lastFrameBl = frame;
        break;
      case AlphaLL_SHOOTER:
        m_lastFrameShooter = frame;
        break;
    }
  }

  private double setxystdev(double distance, double numberOfTags, String name) {
    double xyStdv = 0;
    if (name.equals(AlphaLL_BR)) {
      xyStdv = Math.max(AlphaMINIMUM_XY_STD_DEV_LL3G, (distance * AlphaERROR_FACTOR_LL3G) / numberOfTags);
    } else if (name.equals(AlphaLL_BL) || name.equals(AlphaLL_SHOOTER)) {
      xyStdv = Math.max(AlphaMINIMUM_XY_STD_DEV_LL4, (distance * AlphaERROR_FACTOR_LL4) / numberOfTags);
    } else {
      xyStdv = 0;
    }
    return xyStdv;
  }

  private double setthetastdev(double distance, double numberOfTags, String name) {
    double thetaStdv = 999999999;
    if (name.equals(AlphaLL_BR)) {
      thetaStdv = Math.max(AlphaMINIMUM_THETA_STD_DEV_LL3G, (distance * AlphaERROR_FACTOR_LL3G_ANGLE) / numberOfTags);
    } else if (name.equals(AlphaLL_BL) || name.equals(AlphaLL_SHOOTER)) {
      thetaStdv = Math.max(AlphaMINIMUM_THETA_STD_DEV_LL4, (distance * AlphaERROR_FACTOR_LL4_ANGLE) / numberOfTags);
    }
    return thetaStdv;
  }

  private void updateFusionOdometry() {
    m_swerve.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
    LimelightHelpers.PoseEstimate estimateBL = LimelightHelpers
        .getBotPoseEstimate_wpiBlue_MegaTag2(AlphaLL_BL);
    LimelightHelpers.PoseEstimate estimateBR = LimelightHelpers
        .getBotPoseEstimate_wpiBlue_MegaTag2(AlphaLL_BR);

    LimelightHelpers.PoseEstimate estimateShooter = LimelightHelpers
        .getBotPoseEstimate_wpiBlue_MegaTag2(AlphaLL_SHOOTER);

    record PoseData(PoseEstimate estimate, double frame, String limelightId) {
    }

    PoseData BL = new PoseData(estimateBL, getFrame(AlphaLL_BL), AlphaLL_BL);
    PoseData BR = new PoseData(estimateBR, getFrame(AlphaLL_BR), AlphaLL_BR);
    PoseData SHOOTER = new PoseData(estimateShooter, getFrame(AlphaLL_SHOOTER), AlphaLL_SHOOTER);
    PoseData[] limelights = { BL, BR, SHOOTER };

    for (PoseData limelight : limelights) {
      updateFrame(getFrame(limelight.limelightId()), limelight.limelightId());
      if (!verifyLimelightValidity(limelight.estimate(),
          getFrame(limelight.limelightId()))) {
        continue;
      }
      double numberOfTags = limelight.estimate().tagCount;
      double distance = limelight.estimate().avgTagDist;
      double xystdev = setxystdev(distance, numberOfTags, limelight.limelightId());
      double thetastdev = setthetastdev(distance, numberOfTags, limelight.limelightId());

      // check 1st and 2nd argument
      m_swerve.addVisionMeasurement(limelight.estimate().pose,
          limelight.estimate().timestampSeconds, VecBuilder.fill(xystdev, xystdev,
              thetastdev));
    }
  }

  public double getTargetYawFromBr() {
    double[] temp = LimelightHelpers.getTargetPose_RobotSpace(AlphaLL_BR);
    return temp.length == 0 ? 0 : temp[4];
  }

  public double getTargetYawFromBl() {
    double[] temp = LimelightHelpers.getTargetPose_RobotSpace(AlphaLL_BL);
    return temp.length == 0 ? 0 : temp[4];
  }

  public double getTargetYawFromShooter() {
    double[] temp = LimelightHelpers.getTargetPose_RobotSpace(AlphaLL_SHOOTER);
    return temp.length == 0 ? 0 : temp[4];
  }

  private double getFrame(String limelight) {
    return NetworkTableInstance.getDefault().getTable(limelight).getEntry("hb").getDouble(-1);
  }

  // find alternative to this function during testing. setThrottle no longer
  // exists
  private void ManageLimelightTemperature() {
    m_slow_count++;
    if (m_enabled && (m_slow_count % 50) != 0) {
      return;
    }
    // update at 1Hz or when disabled
    m_enabled = DriverStation.isEnabled();
    final int throttle = m_enabled ? 0 : 169;
    // Apply throttle to all five configured Limelights
    // LimelightHelpers.SetThrottle(LimelightConstants.LL_SHOOTER, throttle);
    // LimelightHelpers.SetThrottle(LimelightConstants.LL_BR, throttle);
    // LimelightHelpers.SetThrottle(LimelightConstants.LL_BL, throttle);
  }

  public Command AlphaLimelightDefaultCommand() {
    return run(() -> {

    });
  }

  @Override
  public void periodic() {
    ManageLimelightTemperature();
    // read the three Limelight tables and store whether each currently sees a
    // target
    m_has_shooter_tag = LimelightHelpers.getTV(AlphaLL_SHOOTER);
    m_has_br_tag = LimelightHelpers.getTV(AlphaLL_BR);
    m_has_bl_tag = LimelightHelpers.getTV(AlphaLL_BL);

    // changed - when enabled configure IMU mode for all Limelights we use
    // if (m_enabled) {
    // LimelightHelpers.SetIMUMode(AlphaLL_SHOOTER, 1);
    // LimelightHelpers.SetIMUMode(AlphaLL_BL, 1);
    // }

    if (m_swerve == null) {
      return;
    }

    // dummy value until swerve is added
    final var yaw_degrees = AlphaSubsystem.swerve.getYawDegrees();

    // according to limelight docs, this needs to be called before using
    // .getBotPoseEstimate_wpiBlue_MegaTag2
    // supply current robot orientation to every Limelight before asking for pose
    // estimates
    LimelightHelpers.SetRobotOrientation(AlphaLL_SHOOTER, yaw_degrees, 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation(AlphaLL_BR, yaw_degrees, 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation(AlphaLL_BL, yaw_degrees, 0, 0, 0, 0, 0);
    updateFusionOdometry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}