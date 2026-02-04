// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;
import frc.robot.util.Subsystems2026;
import frc.robot.util.Constants;
import static frc.robot.util.Constants.LimelightConstants.*;

//Uncommment everything with swerve in it when swerve is added

public class Vision extends SubsystemBase {
  LimelightHelpers m_limelightHelpers;
  double m_robotYaw;
  private final Limelight m_fl;
  private final Limelight m_fr;
  private final Limelight m_bl;
  private final Limelight m_br;
  // Swerve m_swerve;
  Rotation2d m_targetYaw;
  private boolean m_has_shooter_tag;
  private boolean m_has_fr_tag;
  private boolean m_has_fl_tag;
  private boolean m_has_br_tag;
  private boolean m_has_bl_tag;
  private boolean m_enabled = false;
  private long m_slow_count = 0;

  public Vision() {
    // m_swerve = Subsystem.swerve;
    m_limelightHelpers = new LimelightHelpers();
    m_fl = new Limelight(LL_FL, 4);
    m_fr = new Limelight(LL_FR, 3);
    m_bl = new Limelight(LL_BL, 4);
    m_br = new Limelight(LL_BR, 3.5);
    LimelightHelpers.setLEDMode_ForceOff(LL_BL);
    LimelightHelpers.setCameraPose_RobotSpace(LL_BL,
        BL_FORWARD_OFFSET, // Forward offset (meters)
        BL_SIDE_OFFSET, // Side offset (meters) left is positive
        BL_HEIGHT_OFFSET, // Height offset (meters)
        BL_ROLL_OFFSET, // Roll (degrees)
        BL_PITCH_OFFSET, // Pitch (degrees)
        BL_YAW_OFFSET // Yaw (degrees)
    );
    LimelightHelpers.SetFiducialDownscalingOverride(LL_BL, 3);

    LimelightHelpers.setLEDMode_ForceOff(LL_BR);
    LimelightHelpers.setCameraPose_RobotSpace(LL_BR,
        BR_FORWARD_OFFSET, // Forward offset (meters)
        BR_SIDE_OFFSET, // Side offset (meters) left is positive
        BR_HEIGHT_OFFSET, // Height offset (meters)
        BR_ROLL_OFFSET, // Roll (degrees)
        BR_PITCH_OFFSET, // Pitch (degrees)
        BR_YAW_OFFSET // Yaw (degrees)
    );
    LimelightHelpers.SetFiducialDownscalingOverride(LL_BR, 3);

    LimelightHelpers.setLEDMode_ForceOff(LL_FL);
    LimelightHelpers.setCameraPose_RobotSpace(LL_FL,
        FL_FORWARD_OFFSET, // Forward offset (meters)
        FL_SIDE_OFFSET, // Side offset (meters) left is positive
        FL_HEIGHT_OFFSET, // Height offset (meters)
        FL_ROLL_OFFSET, // Roll (degrees)
        FL_PITCH_OFFSET, // Pitch (degrees)
        FL_YAW_OFFSET // Yaw (degrees)
    );

    LimelightHelpers.SetFiducialDownscalingOverride(LL_FL, 3);
    LimelightHelpers.setLEDMode_ForceOff(LL_FR);
    LimelightHelpers.setCameraPose_RobotSpace(LL_FR,
        FR_FORWARD_OFFSET, // Forward offset (meters)
        FR_SIDE_OFFSET, // Side offset (meters) left is positive
        FR_HEIGHT_OFFSET, // Height offset (meters)
        FR_ROLL_OFFSET, // Roll (degrees)
        FR_PITCH_OFFSET, // Pitch (degrees)
        FR_YAW_OFFSET // Yaw (degrees)
    );

    LimelightHelpers.SetIMUMode(LL_FR, 1);
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

  /** True when the front-right camera currently sees a fiducial. */
  public boolean hasFrontRightTag() {
    return m_has_fr_tag;
  }

  /** True when the front-left camera currently sees a fiducial. */
  public boolean hasFrontLeftTag() {
    return m_has_fl_tag;
  }

  /** True when the back-right camera currently sees a fiducial. */
  public boolean hasBackRightTag() {
    return m_has_br_tag;
  }

  /** True when the back-left camera currently sees a fiducial. */
  public boolean hasBackLeftTag() {
    return m_has_bl_tag;
  }

  public double getTargetYawFromFr() {
    double[] temp = LimelightHelpers.getTargetPose_RobotSpace(LL_FR);
    return temp.length == 0 ? 0 : temp[4];
  }

  public double getTargetYawFromFl() {
    double[] temp = LimelightHelpers.getTargetPose_RobotSpace(LL_FL);
    return temp.length == 0 ? 0 : temp[4];
  }

  public double getTargetYawFromBr() {
    double[] temp = LimelightHelpers.getTargetPose_RobotSpace(LL_BR);
    return temp.length == 0 ? 0 : temp[4];
  }

  public double getTargetYawFromBl() {
    double[] temp = LimelightHelpers.getTargetPose_RobotSpace(LL_BL);
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
    m_enabled = DriverStation.isEnabled();
    final int throttle = m_enabled ? 0 : 169;
    m_bl.setThrottle(throttle);
    m_br.setThrottle(throttle);
    m_fl.setThrottle(throttle);
  }

  public Command limelightDefaultCommand() {
    return run(() -> {

    });
  }

  @Override
  public void periodic() {
    ManageLimelightTemperature();

    // if (m_swerve == null) {
    // return;
    // }

    // dummy value until swerve is added
    // final var yaw_degrees = Subsystem.swerve.getYawDegrees();
    final var yaw_degrees = 0;

    // according to limelight docs, this needs to be called before using
    // .getBotPoseEstimate_wpiBlue_MegaTag2
    // supply current robot orientation to every Limelight before asking for pose
    m_fl.setRobotOrientation(yaw_degrees);
    m_fr.setRobotOrientation(yaw_degrees);
    m_bl.setRobotOrientation(yaw_degrees);
    m_br.setRobotOrientation(yaw_degrees);

    m_fl.updateFusionOdometry();
    m_fr.updateFusionOdometry();
    m_bl.updateFusionOdometry();
    m_br.updateFusionOdometry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  private class Limelight {

    private final String name;
    private final double model;
    private double lastFrame = -2;
    private double frame = -2;
    private double BumpScaleFactor = 1;
    private PoseEstimate estimate;

    // use external IMU yaw submitted via setRobotOrientation() and configure the
    // LL4 internal IMU's fused yaw to match the submitted yaw value
    // 0 - Use external IMU yaw submitted via SetRobotOrientation() for MT2
    // localization. The internal IMU is ignored entirely.
    // 1 - Use external IMU yaw submitted via SetRobotOrientation(), and configure
    // the LL4 internal IMU's fused yaw to match the submitted yaw value.
    // 2 - Use internal IMU for MT2 localization.
    Limelight(String p_name, double p_model) {
      name = p_name;
      model = p_model;
      if (model == 4) {
        LimelightHelpers.SetIMUMode(name, 1);
      }
    }

    public void setRobotOrientation(double yawDeg) {
      LimelightHelpers.SetRobotOrientation(name, yawDeg, 0, 0, 0, 0, 0);
    }

    public void updateFusionOdometry() {
      estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
      frame = getFrame();

      if (!verifyLimelightValidity()) {
        updateFrame();
        return;
      }

      double numberOfTags = estimate.tagCount;
      double distance = estimate.avgTagDist;

      double xystdev = setxystdev(distance, numberOfTags);
      double thetastdev = setthetastdev(99999999);

      // m_swerve.addVisionMeasurement(
      // estimate.pose,
      // estimate.timestampSeconds,
      // VecBuilder.fill(xystdev, xystdev, thetastdev));

      updateFrame();
    }

    public Pose2d getEstimatePose() {
      if (estimate == null) {
        return new Pose2d();
      }
      return estimate.pose;
    }

    private boolean verifyLimelightValidity() {
      return estimate != null;
      // && estimate.tagCount != 0 // test
      // && m_swerve.getState().Speeds.omegaRadiansPerSecond < Math.PI // maybe change
      // to two depending on max speed
      // && frame > lastFrame
      // && !Double.isNaN(estimate.avgTagDist)
      // && poseInField();
    }

    private void updateFrame() {
      lastFrame = frame;
    }

    private double getErrorFactor() {
      double errorFactor = 2000;
      if (model == 4) {
        errorFactor = ERROR_FACTOR_LL4;
      } else if (model == 3.5) {
        errorFactor = ERROR_FACTOR_LL3G;
      } else if (model == 3) {
        errorFactor = ERROR_FACTOR_LL3;
      }
      return errorFactor;
    }

    private double getMinimumStdDev() {
      double minStdDev = 2000;
      if (model == 4) {
        minStdDev = MINIMUM_XY_STD_DEV_LL4;
      } else if (model == 3.5) {
        minStdDev = MINIMUM_XY_STD_DEV_LL3G;
      } else if (model == 3) {
        minStdDev = MINIMUM_XY_STD_DEV_LL3;
      }
      return minStdDev;
    }

    private double setxystdev(double distance, double numberOfTags) {
      setBumpScaleFactor();
      double xyStdv = 0;
      double errorFactor = getErrorFactor();
      double minimumXyStdDev = getMinimumStdDev();
      xyStdv = Math.max(
          minimumXyStdDev,
          (Math.pow(distance, 3) * errorFactor) * BumpScaleFactor / Math.pow(numberOfTags, 2));
      return xyStdv;
    }

    private double setthetastdev(double stDev) {
      double thetaStdv = stDev;
      return thetaStdv;
    }

    private double getFrame() {
      return NetworkTableInstance.getDefault()
          .getTable(name)
          .getEntry("hb")
          .getDouble(-1);
    }

    public double getTargetYaw() {
      double[] temp = LimelightHelpers.getTargetPose_RobotSpace(name);
      return temp.length == 0 ? 0 : temp[4];
    }

    // test if works
    public void setThrottle(int throttle) {
      NetworkTableInstance.getDefault().getTable(name).getEntry("throttle_set").setNumber(throttle);
    }

    public boolean onBump() {
      return false;
      // return Math.abs(m_swerve.getPitch()) > BUMP_MINIMUM_THRESHOLD;
    }

    private void setBumpScaleFactor() {
      if (onBump()) {
        BumpScaleFactor = .5;
      }
      BumpScaleFactor = 1;
    }

    public boolean poseInField() {
      return estimate.pose.getMeasureX().compareTo(ZERO) >= 0
          && estimate.pose.getMeasureX().compareTo(FIELD_DIMENSION_X) <= 0
          && estimate.pose.getMeasureY().compareTo(ZERO) >= 0
          && estimate.pose.getMeasureY().compareTo(FIELD_DIMENSION_Y) <= 0;
    }
  }
}