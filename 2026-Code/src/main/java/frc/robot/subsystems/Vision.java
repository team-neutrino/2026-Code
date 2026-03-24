// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;
import frc.robot.util.LimelightHelpers.RawFiducial;
import static frc.robot.util.Constants.LimelightConstants.*;
import static frc.robot.util.Constants.SwerveConstants.ROBOT_WHEEL_OFFSET;
import static frc.robot.util.Constants.AprilTagConstants.*;
import static frc.robot.util.Constants.FieldMeasurementConstants.*;
import static frc.robot.util.Subsystems.swerve;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructTopic;

/**
 * Vision subsystem responsible for configuring, managing, validating,
 * and fusing pose data from multiple Limelight cameras into the
 * drivetrain pose estimator.
 *
 * <p>
 * This subsystem:
 * <ul>
 * <li>Configures camera offsets and IMU fusion modes</li>
 * <li>Supplies robot orientation data to Limelights</li>
 * <li>Validates MegaTag1 (MT1) and MegaTag2 (MT2) measurements</li>
 * <li>Dynamically computes measurement covariance</li>
 * <li>Fuses trusted measurements into the swerve estimator</li>
 * <li>Publishes pose/yaw data to NetworkTables</li>
 * <li>Manages thermal throttling while disabled</li>
 * </ul>
 */
public class Vision extends SubsystemBase {

  private final Limelight m_front;
  private final Limelight m_back;
  private final Limelight m_left;
  private final Limelight m_right;

  private boolean m_enabled = false;
  private Timer m_timer = new Timer();

  private Limelight[] limelights;

  private double m_last_vision_update_timestamp = 0;

  /**
   * Constructs the Vision subsystem and initializes all Limelights,
   * publishers, and configuration parameters.
   */
  public Vision() {
    m_front = new Limelight(LL_FRONT, 4);
    m_back = new Limelight(LL_BACK, 4);
    m_left = new Limelight(LL_LEFT, 3);
    m_right = new Limelight(LL_RIGHT, 3.5);
    limelights = new Limelight[] { m_front, m_back, m_left, m_right };

    m_timer.start();

    limelightInitialization();
  }

  /**
   * Performs one-time initialization of all Limelights:
   * sets LED modes, camera pose offsets, fiducial scaling,
   * and IMU fusion configuration.
   */
  private void limelightInitialization() {

    LimelightHelpers.setLEDMode_ForceOff(LL_FRONT);
    LimelightHelpers.setCameraPose_RobotSpace(LL_FRONT,
        FRONT_FORWARD_OFFSET,
        FRONT_SIDE_OFFSET,
        FRONT_HEIGHT_OFFSET,
        FRONT_ROLL_OFFSET,
        FRONT_PITCH_OFFSET,
        FRONT_YAW_OFFSET);
    LimelightHelpers.SetFiducialDownscalingOverride(LL_FRONT, 1);

    LimelightHelpers.setLEDMode_ForceOff(LL_BACK);
    LimelightHelpers.setCameraPose_RobotSpace(LL_BACK,
        BACK_FORWARD_OFFSET,
        BACK_SIDE_OFFSET,
        BACK_HEIGHT_OFFSET,
        BACK_ROLL_OFFSET,
        BACK_PITCH_OFFSET,
        BACK_YAW_OFFSET);
    LimelightHelpers.SetFiducialDownscalingOverride(LL_BACK, 1);

    LimelightHelpers.setLEDMode_ForceOff(LL_LEFT);
    LimelightHelpers.SetFiducialDownscalingOverride(LL_LEFT, 1);
    LimelightHelpers.setCameraPose_RobotSpace(LL_LEFT,
        LEFT_FORWARD_OFFSET,
        LEFT_SIDE_OFFSET,
        LEFT_HEIGHT_OFFSET,
        LEFT_ROLL_OFFSET,
        LEFT_PITCH_OFFSET,
        LEFT_YAW_OFFSET);

    LimelightHelpers.SetFiducialDownscalingOverride(LL_RIGHT, 1);
    LimelightHelpers.setLEDMode_ForceOff(LL_RIGHT);
    LimelightHelpers.setCameraPose_RobotSpace(LL_RIGHT,
        RIGHT_FORWARD_OFFSET,
        RIGHT_SIDE_OFFSET,
        RIGHT_HEIGHT_OFFSET,
        RIGHT_ROLL_OFFSET,
        RIGHT_PITCH_OFFSET,
        RIGHT_YAW_OFFSET);

    for (Limelight limelight : limelights) {
      LimelightHelpers.setPipelineIndex(limelight.name, 0);
      if (limelight.model == 4) {
        LimelightHelpers.setRewindEnabled(limelight.name, true);
        LimelightHelpers.SetIMUAssistAlpha(limelight.name, EXTERNAL_WEIGHT);
      }
    }
  }

  /**
   * Returns the default command for this subsystem.
   *
   * @return empty run command
   */
  public Command limelightDefaultCommand() {
    return run(() -> {
    });
  }

  /**
   * Called once per scheduler cycle.
   *
   * Supplies robot orientation to Limelights,
   * performs fusion updates,
   * manages yaw reseeding,
   * and publishes pose/yaw estimates.
   */
  @Override
  public void periodic() {

    if (swerve == null) {
      return;
    }

    final double yaw_degrees = swerve.getYawDegrees();
    final double pitch_degrees = swerve.getPitch();
    final double roll_degrees = swerve.getRoll();
    final double yaw_rate = swerve.getYawRate();
    final double pitch_rate = swerve.getPitchRate();
    final double roll_rate = swerve.getRollRate();

    boolean better_limelight_found_two_hub_tags = false;
    for (Limelight limelight : limelights) {
      limelight.setRobotOrientation(yaw_degrees, yaw_rate, pitch_degrees, pitch_rate, roll_degrees, roll_rate);
      limelight.adjustIMUMode();
      limelight.triggerCaptureRewind();

      if (better_limelight_found_two_hub_tags) {
        limelight.publishDefaultPose();
        limelight.publishDefaultYaw();
        break;
      }
      limelight.updateFusionMegatag();
      limelight.updatePigeonSeed();
      limelight.publishPose();
      limelight.publishYaw();
      better_limelight_found_two_hub_tags = limelight.hasTwoHubTags();
    }
  }

  @Override
  public void simulationPeriodic() {
  }

  /**
   * Represents a single Limelight camera instance.
   *
   * Encapsulates validation, covariance calculation,
   * yaw reseeding, and measurement fusion logic.
   */
  private class Limelight {

    private final String name;
    private final double model;
    private double lastFrame = -2;
    private PoseEstimate estimateMT1;
    private PoseEstimate estimateMT2;
    private boolean m_rewindTriggered = false;
    private boolean m_updatedImuModeSinceEnabled = false;
    private int m_hubTagCount = 0;

    private NetworkTableInstance m_nt = NetworkTableInstance.getDefault();
    private StructTopic<Pose2d> m_pose;
    private StructPublisher<Pose2d> m_posePub;
    private Pose2d blank = new Pose2d();
    private DoubleTopic m_yaw;
    private DoublePublisher m_yawPub;
    private boolean m_poseZeroWasPublished = false;
    private boolean m_yawZeroWasPublished = false;

    /**
     * Constructs a Limelight wrapper.
     *
     * @param p_name  NetworkTables name
     * @param p_model Limelight hardware model
     */
    Limelight(String p_name, double p_model) {
      name = p_name;
      model = p_model;
      if (model == 4) {
        LimelightHelpers.SetIMUMode(name, 1);
      }
      m_pose = m_nt.getStructTopic("/limelight_poses/" + name, Pose2d.struct);
      m_yaw = m_nt.getDoubleTopic("/limelight_poses/yaw/" + name + "Yaw");

      m_posePub = m_pose.publish();
      m_posePub.setDefault(blank);

      m_yawPub = m_yaw.publish(PubSubOption.keepDuplicates(false));
    }

    public void publishPose() {
      if (!m_poseZeroWasPublished && this.getEstimatePose().equals(Pose2d.kZero)) {
        m_posePub.set(this.getEstimatePose());
        m_poseZeroWasPublished = true;
      } else if (!this.getEstimatePose().equals(Pose2d.kZero)) {
        m_posePub.set(this.getEstimatePose());
        m_poseZeroWasPublished = false;
      }
    }

    public void publishYaw() {
      if (!m_yawZeroWasPublished && this.getEstimateYawMT1() == IGNORE_MEASUREMENT_STD_DEV) {
        m_yawPub.set(this.getEstimateYawMT1());
        m_yawZeroWasPublished = true;
      } else if (this.getEstimateYawMT1() != IGNORE_MEASUREMENT_STD_DEV) {
        m_yawPub.set(this.getEstimateYawMT1());
        m_yawZeroWasPublished = false;
      }
    }

    public void publishDefaultPose() {
      if (!m_poseZeroWasPublished) {
        m_posePub.set(Pose2d.kZero);
        m_poseZeroWasPublished = true;
      }
    }

    public void publishDefaultYaw() {
      if (!m_yawZeroWasPublished) {
        m_yawPub.set(IGNORE_MEASUREMENT_STD_DEV);
        m_yawZeroWasPublished = true;
      }
    }

    public boolean hasTwoHubTags() {
      return m_hubTagCount >= 2;
    }

    /** Supplies robot orientation to the Limelight for IMU fusion. */
    public void setRobotOrientation(double yawDeg, double yawRate, double pitchDeg,
        double pitchRate, double rollDeg, double rollRate) {
      LimelightHelpers.SetRobotOrientation(name, yawDeg, yawRate, pitchDeg, pitchRate, rollDeg, rollRate);

    }

    /**
     * Retrieves MT1 and MT2 pose estimates, validates them,
     * selects the best available measurement,
     * and fuses into the drivetrain estimator.
     */
    public void updateFusionMegatag() {
      final double frame = getFrame();
      if (frame <= lastFrame || frame < 0.0) {
        return;
      }
      lastFrame = frame;

      estimateMT1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
      estimateMT2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
      updateHubTagCount(estimateMT2);

      double timestamp;
      Pose2d pose;
      if (!verifyPoseValidity()) {
        return;
      }
      if (!verifyYawValidity()) {
        timestamp = estimateMT2.timestampSeconds;
        pose = estimateMT2.pose;
      } else {
        pose = new Pose2d(
            estimateMT2.pose.getTranslation(),
            estimateMT1.pose.getRotation());
        timestamp = estimateMT2.timestampSeconds;
      }

      if (m_last_vision_update_timestamp < timestamp) {
        m_last_vision_update_timestamp = timestamp;
        swerve.addVisionMeasurement(pose, timestamp,
            VecBuilder.fill(getCalcXYStdev(), getCalcXYStdev(), getCalcYawStdev()));
      }
    }

    /** @return Latest MT2 pose or blank pose if unavailable. */
    public Pose2d getEstimatePose() {
      if (estimateMT2 == null) {
        return Pose2d.kZero;
      }
      return estimateMT2.pose;
    }

    /** @return Latest MT1 yaw in degrees or ignore value if invalid. */
    public double getEstimateYawMT1() {
      if (estimateMT1 == null) {
        return IGNORE_MEASUREMENT_STD_DEV;
      }
      return estimateMT1.pose.getRotation().getDegrees();
    }

    /** Determines whether MT2 pose measurement is valid for fusion. */
    private boolean verifyPoseValidity() {
      return estimateMT2 != null
          && estimateMT2.tagCount != 0
          && Math.abs(swerve.getState().Speeds.omegaRadiansPerSecond) < Math.PI
          && !Double.isNaN(estimateMT2.avgTagDist)
          && poseInField(estimateMT2);
    }

    /** Determines whether MT1 yaw measurement is valid for fusion. */
    private boolean verifyYawValidity() {
      return estimateMT1 != null
          && estimateMT1.tagCount > 1
          && Math.abs(swerve.getState().Speeds.omegaRadiansPerSecond) < Math.PI / 2
          && poseInField(estimateMT1);
    }

    /** Determines whether conditions are safe for yaw reseeding. */
    private boolean verifyPigeonSeedUpdate() {
      return estimateMT1 != null
          && estimateMT1.tagCount > 1
          && Math.abs(swerve.getState().Speeds.omegaRadiansPerSecond) < Math.PI / 4
          && poseInField(estimateMT1)
          && swerve.getSpeedMetersPerSecond() < PIGEON_SEED_XY_THRESHOLD
          && m_timer.hasElapsed(PIGEON_SEED_PERIOD);
    }

    /** Seeds drivetrain yaw using MT1 measurement if conditions allow. */
    public void updatePigeonSeed() {
      if (verifyPigeonSeedUpdate()) {
        swerve.seedYawMT1(estimateMT1.pose.getRotation().getDegrees(), MT1_WEIGHT_YAW);
        m_timer.restart();
      }
    }

    /** Calculates XY measurement standard deviation dynamically. */
    private double getCalcXYStdev() {
      if (!verifyPoseValidity()) {
        return IGNORE_MEASUREMENT_STD_DEV;
      }
      double numberOfTags = estimateMT2.tagCount;
      double distance = estimateMT2.avgTagDist;
      return setXYstdev(distance, numberOfTags, m_hubTagCount);
    }

    /** Calculates rotational (theta) measurement standard deviation dynamically. */
    private double getCalcYawStdev() {
      if (!verifyYawValidity()) {
        return IGNORE_MEASUREMENT_STD_DEV;
      }
      double distance = estimateMT1.avgTagDist;
      return setThetastdev(distance, m_hubTagCount);
    }

    /** Returns error factor constant based on Limelight model. */
    private double getErrorFactor() {
      double errorFactor = 2000;
      if (model == 4)
        errorFactor = ERROR_FACTOR_LL4;
      else if (model == 3.5)
        errorFactor = ERROR_FACTOR_LL3G;
      else if (model == 3)
        errorFactor = ERROR_FACTOR_LL3;
      return errorFactor;
    }

    /** Returns minimum allowed XY standard deviation for model. */
    private double getMinimumStdDev() {
      double minStdDev = 2000;
      if (model == 4)
        minStdDev = MINIMUM_XY_STD_DEV_LL4;
      else if (model == 3.5)
        minStdDev = MINIMUM_XY_STD_DEV_LL3G;
      else if (model == 3)
        minStdDev = MINIMUM_XY_STD_DEV_LL3;
      return minStdDev;
    }

    /** Returns minimum allowed rotational standard deviation for model. */
    private double getMinimumStdDevTheta() {
      double minStdDev = 2000;
      if (model == 4)
        minStdDev = MINIMUM_THETA_STD_DEV_LL4;
      else if (model == 3.5)
        minStdDev = MINIMUM_THETA_STD_DEV_LL3G;
      else if (model == 3)
        minStdDev = MINIMUM_THETA_STD_DEV_LL3;
      return minStdDev;
    }

    /** Computes XY standard deviation using tag distance and count. */
    private double setXYstdev(double distance, double numberOfTags, int numberOfHubTags) {
      double errorFactor = getErrorFactor();
      double minimumXyStdDev = getMinimumStdDev();
      if (numberOfHubTags <= 2) {
        errorFactor *= 10.0;
        minimumXyStdDev *= 10.0;
      }

      if (onBump()) {
        return minimumXyStdDev;
      }

      return Math.max(
          minimumXyStdDev,
          (Math.pow(distance, 2) * errorFactor) / Math.pow(numberOfTags, 2));
    }

    /** Computes rotational standard deviation using tag distance. */
    private double setThetastdev(double distance, int numberOfHubTags) {
      if (!verifyYawValidity()) {
        return IGNORE_MEASUREMENT_STD_DEV;
      }
      double errorFactor = getErrorFactor();
      double minimumThetaStDev = getMinimumStdDevTheta();
      if (numberOfHubTags <= 2) {
        errorFactor *= 10.0;
        minimumThetaStDev *= 10.0;
      }

      return Math.max(minimumThetaStDev, (Math.pow(distance, 2) * errorFactor));
    }

    /** Returns current Limelight frame heartbeat value. */
    private double getFrame() {
      return NetworkTableInstance.getDefault()
          .getTable(name)
          .getEntry("hb")
          .getDouble(-1);
    }

    /** Sets processing throttle for this Limelight. */
    public void setThrottle(int throttle) {
      NetworkTableInstance.getDefault().getTable(name)
          .getEntry("throttle_set")
          .setNumber(throttle);
    }

    /** @return true if robot pitch exceeds bump threshold. */
    public boolean onBump() {
      double PoseX = swerve.getCurrentPose().getX();
      double PoseY = swerve.getCurrentPose().getY();
      boolean isXOnBump = ((PoseX >= BLUE_DEPOT_BUMP_ALLIANCE_X - ROBOT_WHEEL_OFFSET)
          && (PoseX <= BLUE_DEPOT_BUMP_NEUTRAL_X + ROBOT_WHEEL_OFFSET))
          || ((PoseX <= RED_DEPOT_BUMP_ALLIANCE_X + ROBOT_WHEEL_OFFSET)
              && (PoseX >= RED_DEPOT_BUMP_NEUTRAL_X - ROBOT_WHEEL_OFFSET));
      boolean isYOnBump = (DEPOT_BUMP_Y >= PoseY) && (OUTPOST_BUMP_Y <= PoseY);
      return isYOnBump && isXOnBump;
    }

    /** Adjusts IMU fusion mode dynamically based on enable state. */

    // 0 EXTERNAL_ONLY External (NT/HTTP) No internal IMU processing. MT2 uses
    // interpolated yaw from robot's gyro sent via SetRobotOrientation().
    // 1 EXTERNAL_SEED External (NT/HTTP) Internal IMU offset is calibrated to match
    // external yaw each frame (seeding). MT2 still uses external yaw for botpose.
    // 2 INTERNAL_ONLY Internal IMU Uses internal IMU's fused yaw only. No external
    // input required.
    // 3 INTERNAL_MT1_ASSIST Internal IMU + MT1 Complementary filter fuses internal
    // IMU with MT1 vision yaw. When MT1 gets a valid pose, it slowly corrects
    // internal IMU drift.
    // 4 INTERNAL_EXTERNAL_ASSIST Internal IMU + External IMU Complementary filter
    // fuses internal IMU with external yaw from SetRobotOrientation(). This is the
    // recommended mode, as the internal IMU's 1khz update rate is utilized for
    // frame-by-frame motion while the robot's IMU corrects for any drift over time.

    public void adjustIMUMode() {
      if (!m_enabled) {
        m_updatedImuModeSinceEnabled = false;
        LimelightHelpers.SetIMUMode(name, 1);
      } else if (model == 4 && !m_updatedImuModeSinceEnabled) {
        LimelightHelpers.SetIMUMode(name, 4);
        m_updatedImuModeSinceEnabled = true;
      }
    }

    /** Triggers capture rewind for LL4 cameras at match start. */
    public void triggerCaptureRewind() {
      if (model == 4 && DriverStation.getMatchTime() <= 1.0 && !m_rewindTriggered) {
        LimelightHelpers.triggerRewindCapture(name, 165);
        m_rewindTriggered = true;
      }
    }

    /** Validates that a pose lies within official field boundaries. */
    public boolean poseInField(PoseEstimate poseEstimate) {
      if (poseEstimate == null || poseEstimate.pose.getTranslation().equals(Translation2d.kZero)) {
        return false;
      }
      return poseEstimate.pose.getX() > ZERO
          && poseEstimate.pose.getX() < FIELD_DIMENSION_X
          && poseEstimate.pose.getY() > ZERO
          && poseEstimate.pose.getY() < FIELD_DIMENSION_Y;
    }

    private void updateHubTagCount(PoseEstimate estimate) {
      if (estimate == null) {
        m_hubTagCount = 0;
        return;
      }

      int hubTagCount = 0;
      for (RawFiducial fiducial : estimate.rawFiducials) {
        if (ALL_HUB_TAGS.contains(fiducial.id)) {
          hubTagCount++;
        }
      }
      m_hubTagCount = hubTagCount;
    }
  }
}
