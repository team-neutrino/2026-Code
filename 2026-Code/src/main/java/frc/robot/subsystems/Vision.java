// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
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
import static frc.robot.util.Constants.LimelightConstants.*;
import static frc.robot.util.Subsystems.swerve;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructTopic;

public class Vision extends SubsystemBase {
  private final Limelight m_front;
  private final Limelight m_back;
  private final Limelight m_left;
  private final Limelight m_right;
  private boolean m_enabled = false;
  private long m_slow_count = 0;
  private Timer m_timer = new Timer();

  private NetworkTableInstance m_nt = NetworkTableInstance.getDefault();
  private StructTopic<Pose2d> m_frontPose = m_nt.getStructTopic("/limelight_poses/front", Pose2d.struct);
  private StructTopic<Pose2d> m_backPose = m_nt.getStructTopic("/limelight_poses/back", Pose2d.struct);
  private StructTopic<Pose2d> m_leftPose = m_nt.getStructTopic("/limelight_poses/left", Pose2d.struct);
  private StructTopic<Pose2d> m_rightPose = m_nt.getStructTopic("/limelight_poses/right", Pose2d.struct);
  private StructPublisher<Pose2d> m_frontPosePub;
  private StructPublisher<Pose2d> m_backPosePub;
  private StructPublisher<Pose2d> m_leftPosePub;
  private StructPublisher<Pose2d> m_rightPosePub;
  private Pose2d blank = new Pose2d();
  private DoubleTopic m_frontYaw = m_nt.getDoubleTopic("/limelight_poses/yaw/frontYaw");
  private DoubleTopic m_backYaw = m_nt.getDoubleTopic("/limelight_poses/yaw/backYaw");
  private DoubleTopic m_leftYaw = m_nt.getDoubleTopic("/limelight_poses/yaw/leftYaw");
  private DoubleTopic m_rightYaw = m_nt.getDoubleTopic("/limelight_poses/yaw/rightYaw");
  private DoublePublisher m_frontYawPub;
  private DoublePublisher m_backYawPub;
  private DoublePublisher m_leftYawPub;
  private DoublePublisher m_rightYawPub;
  private Limelight[] limelights;

  public Vision() {
    m_front = new Limelight(LL_FRONT, 4);
    m_back = new Limelight(LL_BACK, 4);
    m_left = new Limelight(LL_LEFT, 3);
    m_right = new Limelight(LL_RIGHT, 3.5);

    m_frontPosePub = m_frontPose.publish();
    m_frontPosePub.setDefault(blank);
    m_backPosePub = m_backPose.publish();
    m_backPosePub.setDefault(blank);
    m_leftPosePub = m_leftPose.publish();
    m_leftPosePub.setDefault(blank);
    m_rightPosePub = m_rightPose.publish();
    m_rightPosePub.setDefault(blank);

    m_frontYawPub = m_frontYaw.publish(PubSubOption.keepDuplicates(false));
    m_backYawPub = m_backYaw.publish(PubSubOption.keepDuplicates(false));
    m_leftYawPub = m_leftYaw.publish(PubSubOption.keepDuplicates(false));
    m_rightYawPub = m_rightYaw.publish(PubSubOption.keepDuplicates(false));

    limelightInitialization();
    limelights = new Limelight[] { m_front, m_back, m_left, m_right };
  }

  private void limelightInitialization() {

    LimelightHelpers.setLEDMode_ForceOff(LL_FRONT);
    LimelightHelpers.setCameraPose_RobotSpace(LL_FRONT,
        FRONT_FORWARD_OFFSET, // Forward offset (meters)
        FRONT_SIDE_OFFSET, // Side offset (meters) left is positive
        FRONT_HEIGHT_OFFSET, // Height offset (meters)
        FRONT_ROLL_OFFSET, // Roll (degrees)
        FRONT_PITCH_OFFSET, // Pitch (degrees)
        FRONT_YAW_OFFSET // Yaw (degrees)
    );
    LimelightHelpers.SetFiducialDownscalingOverride(LL_FRONT, 3);

    LimelightHelpers.setLEDMode_ForceOff(LL_BACK);
    LimelightHelpers.setCameraPose_RobotSpace(LL_BACK,
        BACK_FORWARD_OFFSET, // Forward offset (meters)
        BACK_SIDE_OFFSET, // Side offset (meters) left is positive
        BACK_HEIGHT_OFFSET, // Height offset (meters)
        BACK_ROLL_OFFSET, // Roll (degrees)
        BACK_PITCH_OFFSET, // Pitch (degrees)
        BACK_YAW_OFFSET // Yaw (degrees)
    );
    LimelightHelpers.SetFiducialDownscalingOverride(LL_LEFT, 3);

    LimelightHelpers.setLEDMode_ForceOff(LL_LEFT);
    LimelightHelpers.setCameraPose_RobotSpace(LL_LEFT,
        LEFT_FORWARD_OFFSET, // Forward offset (meters)
        LEFT_SIDE_OFFSET, // Side offset (meters) left is positive
        LEFT_HEIGHT_OFFSET, // Height offset (meters)
        LEFT_ROLL_OFFSET, // Roll (degrees)
        LEFT_PITCH_OFFSET, // Pitch (degrees)
        LEFT_YAW_OFFSET // Yaw (degrees)
    );

    LimelightHelpers.SetFiducialDownscalingOverride(LL_RIGHT, 3);
    LimelightHelpers.setLEDMode_ForceOff(LL_RIGHT);
    LimelightHelpers.setCameraPose_RobotSpace(LL_RIGHT,
        RIGHT_FORWARD_OFFSET, // Forward offset (meters)
        RIGHT_SIDE_OFFSET, // Side offset (meters) left is positive
        RIGHT_HEIGHT_OFFSET, // Height offset (meters)
        RIGHT_ROLL_OFFSET, // Roll (degrees)
        RIGHT_PITCH_OFFSET, // Pitch (degrees)
        RIGHT_YAW_OFFSET // Yaw (degrees)
    );
  }

  private void manageLimelightTemperature() {
    m_slow_count++;
    if (m_enabled && (m_slow_count % 50) != 0) {
      return;
    }
    m_enabled = DriverStation.isEnabled();
    final int throttle = m_enabled ? 0 : 169;
    m_front.setThrottle(throttle);
    m_back.setThrottle(throttle);
    m_left.setThrottle(throttle);
    m_right.setThrottle(throttle);
  }

  public Command limelightDefaultCommand() {
    return run(() -> {

    });
  }

  @Override
  public void periodic() {
    manageLimelightTemperature();

    if (swerve == null) {
      return;
    }

    final double yaw_degrees = swerve.getYawDegrees();
    final double pitch_degrees = swerve.getPitch();
    final double roll_degrees = swerve.getRoll();
    final double yaw_rate = swerve.getYawRate();
    final double pitch_rate = swerve.getPitchRate();
    final double roll_rate = swerve.getRollRate();

    // according to limelight docs, this needs to be called before using
    // .getBotPoseEstimate_wpiBlue_MegaTag2
    // supply current robot orientation to every Limelight before asking for pose

    for (Limelight limelight : limelights) {
      limelight.setRobotOrientation(yaw_degrees, pitch_degrees, roll_degrees, yaw_rate, pitch_rate, roll_rate);
      limelight.updateFusionMegatag();
      limelight.updatePigeonSeed();
      limelight.adjustIMUMode();
    }

    m_frontPosePub.set(m_front.getEstimatePose());
    m_backPosePub.set(m_back.getEstimatePose());
    m_leftPosePub.set(m_left.getEstimatePose());
    m_rightPosePub.set(m_right.getEstimatePose());

    m_frontYawPub.set(m_front.getEstimateYawMT1());
    m_backYawPub.set(m_back.getEstimateYawMT1());
    m_leftYawPub.set(m_left.getEstimateYawMT1());
    m_rightYawPub.set(m_right.getEstimateYawMT1());
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
    private double bumpScaleFactor = 1;
    private PoseEstimate estimateMT1;
    private PoseEstimate estimateMT2;

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

    Limelight(String p_name, double p_model) {
      name = p_name;
      model = p_model;
      if (model == 4) {
        LimelightHelpers.SetIMUMode(name, 1);
      }
    }

    public void setRobotOrientation(double yawDeg, double pitchDeg, double rollDeg, double yawRate, double pitchRate,
        double rollRate) {
      LimelightHelpers.SetRobotOrientation(name, yawDeg, pitchDeg, rollDeg, yawRate, pitchRate, rollRate);
    }

    private double getCalcXYStdev() {
      frame = getFrame();

      if (!verifyPoseValidity()) {
        updateFrame();
        return IGNORE_MEASUREMENT_STD_DEV;
      }

      double numberOfTags = estimateMT2.tagCount;
      double distance = estimateMT2.avgTagDist;
      updateFrame();
      return setXYstdev(distance, numberOfTags);
    }

    private double getCalcYawStdev() {

      if (!verifyYawValidity()) {
        return IGNORE_MEASUREMENT_STD_DEV;
      }
      double distance = estimateMT1.avgTagDist;
      return setThetastdev(distance);
    }

    public void updateFusionMegatag() {
      double timestamp;
      Pose2d pose;
      if (!verifyPoseValidity() && !verifyYawValidity()) {
        return;
      } else if (!verifyPoseValidity() && verifyYawValidity()) {
        timestamp = estimateMT1.timestampSeconds;
        pose = estimateMT1.pose;
      } else if (verifyPoseValidity() && !verifyYawValidity()) {
        timestamp = estimateMT2.timestampSeconds;
        pose = estimateMT2.pose;
      } else {
        pose = new Pose2d(
            estimateMT2.pose.getTranslation(),
            estimateMT1.pose.getRotation());
        timestamp = Math.max(estimateMT1.timestampSeconds, estimateMT2.timestampSeconds);
      }
      swerve.addVisionMeasurement(pose, timestamp,
          VecBuilder.fill(getCalcXYStdev(), getCalcXYStdev(), getCalcYawStdev()));
    }

    public Pose2d getEstimatePose() {
      if (estimateMT2 == null) {
        return new Pose2d();
      }
      return estimateMT2.pose;
    }

    public double getEstimateYawMT1() {
      if (estimateMT1 == null) {
        // placeholder value so we know it's wrong and we don't have
        return IGNORE_MEASUREMENT_STD_DEV;
      }
      return estimateMT1.pose.getRotation().getDegrees();
    }

    private boolean verifyPoseValidity() {
      return estimateMT2 != null
          && estimateMT2.tagCount != 0
          && swerve.getState().Speeds.omegaRadiansPerSecond < Math.PI // maybe change to two depending on max speed
          && frame > lastFrame
          && !Double.isNaN(estimateMT2.avgTagDist)
          && poseInField(estimateMT2);
    }

    private boolean verifyYawValidity() {
      return estimateMT1 != null && estimateMT1.tagCount > 1
          && swerve.getState().Speeds.omegaRadiansPerSecond < Math.PI / 2 && poseInField(estimateMT1);
    }

    private boolean verifyPigeonSeedUpdate() {
      return estimateMT1 != null && estimateMT1.tagCount > 1
          && swerve.getState().Speeds.omegaRadiansPerSecond < Math.PI / 4 && poseInField(estimateMT1)
          && m_timer.hasElapsed(PIGEON_SEED_PERIOD);
    }

    public void updatePigeonSeed() {
      if (verifyPigeonSeedUpdate()) {
        swerve.seedYawMT1(estimateMT1.pose.getRotation().getRadians(), MT1_WEIGHT_YAW);
        m_timer.restart();
      }
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

    private double getMinimumStdDevTheta() {
      double minStdDev = 2000;
      if (model == 4) {
        minStdDev = MINIMUM_THETA_STD_DEV_LL4;
      } else if (model == 3.5) {
        minStdDev = MINIMUM_THETA_STD_DEV_LL3G;
      } else if (model == 3) {
        minStdDev = MINIMUM_THETA_STD_DEV_LL3;
      }
      return minStdDev;
    }

    private double setXYstdev(double distance, double numberOfTags) {
      setBumpScaleFactor();
      double xyStdv = 0;
      double errorFactor = getErrorFactor();
      double minimumXyStdDev = getMinimumStdDev();
      xyStdv = Math.max(
          minimumXyStdDev,
          (Math.pow(distance, 3) * errorFactor) * bumpScaleFactor / Math.pow(numberOfTags, 2));
      return xyStdv;
    }

    private double setThetastdev(double distance) {
      if (!verifyYawValidity()) {
        return IGNORE_MEASUREMENT_STD_DEV;
      }
      double errorFactor = getErrorFactor();
      double minimumThetaStDev = getMinimumStdDevTheta();
      double thetaStdv = Math.max(minimumThetaStDev, (Math.pow(distance, 2) * errorFactor));
      return thetaStdv;
    }

    private double getFrame() {
      return NetworkTableInstance.getDefault()
          .getTable(name)
          .getEntry("hb")
          .getDouble(-1);
    }

    public void setThrottle(int throttle) {
      NetworkTableInstance.getDefault().getTable(name).getEntry("throttle_set").setNumber(throttle);
    }

    public boolean onBump() {
      return Math.abs(swerve.getPitch()) > BUMP_MINIMUM_THRESHOLD;
    }

    private void setBumpScaleFactor() {
      bumpScaleFactor = onBump() ? 0.5 : 1;
    }

    public void adjustIMUMode() {
      if (model == 4) {
        LimelightHelpers.SetIMUMode(name, m_enabled ? 4 : 1);
      }
    }

    public boolean poseInField(PoseEstimate poseEstimate) {
      if (poseEstimate == null) {
        return false;
      }
      return poseEstimate.pose.getMeasureX().compareTo(ZERO) >= 0
          && poseEstimate.pose.getMeasureX().compareTo(FIELD_DIMENSION_X) <= 0
          && poseEstimate.pose.getMeasureY().compareTo(ZERO) >= 0
          && poseEstimate.pose.getMeasureY().compareTo(FIELD_DIMENSION_Y) <= 0;
    }
  }
}