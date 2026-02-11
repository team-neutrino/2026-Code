// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.alpha_subsystems;

import static frc.robot.util.Constants.AlphabotLimelightConstants.AlphaBL_FORWARD_OFFSET;
import static frc.robot.util.Constants.AlphabotLimelightConstants.AlphaBL_HEIGHT_OFFSET;
import static frc.robot.util.Constants.AlphabotLimelightConstants.AlphaBL_PITCH_OFFSET;
import static frc.robot.util.Constants.AlphabotLimelightConstants.AlphaBL_ROLL_OFFSET;
import static frc.robot.util.Constants.AlphabotLimelightConstants.AlphaBL_SIDE_OFFSET;
import static frc.robot.util.Constants.AlphabotLimelightConstants.AlphaBL_YAW_OFFSET;
import static frc.robot.util.Constants.AlphabotLimelightConstants.AlphaBR_FORWARD_OFFSET;
import static frc.robot.util.Constants.AlphabotLimelightConstants.AlphaBR_HEIGHT_OFFSET;
import static frc.robot.util.Constants.AlphabotLimelightConstants.AlphaBR_PITCH_OFFSET;
import static frc.robot.util.Constants.AlphabotLimelightConstants.AlphaBR_ROLL_OFFSET;
import static frc.robot.util.Constants.AlphabotLimelightConstants.AlphaBR_SIDE_OFFSET;
import static frc.robot.util.Constants.AlphabotLimelightConstants.AlphaBR_YAW_OFFSET;
import static frc.robot.util.Constants.AlphabotLimelightConstants.AlphaERROR_FACTOR_LL3G;
import static frc.robot.util.Constants.AlphabotLimelightConstants.AlphaERROR_FACTOR_LL4;
import static frc.robot.util.Constants.AlphabotLimelightConstants.AlphaLL_BL;
import static frc.robot.util.Constants.AlphabotLimelightConstants.AlphaLL_BR;
import static frc.robot.util.Constants.AlphabotLimelightConstants.AlphaLL_SHOOTER;
import static frc.robot.util.Constants.AlphabotLimelightConstants.AlphaMINIMUM_XY_STD_DEV_LL3G;
import static frc.robot.util.Constants.AlphabotLimelightConstants.AlphaMINIMUM_XY_STD_DEV_LL4;
import static frc.robot.util.Constants.AlphabotLimelightConstants.AlphaSHOOTER_FORWARD_OFFSET;
import static frc.robot.util.Constants.AlphabotLimelightConstants.AlphaSHOOTER_HEIGHT_OFFSET;
import static frc.robot.util.Constants.AlphabotLimelightConstants.AlphaSHOOTER_PITCH_OFFSET;
import static frc.robot.util.Constants.AlphabotLimelightConstants.AlphaSHOOTER_ROLL_OFFSET;
import static frc.robot.util.Constants.AlphabotLimelightConstants.AlphaSHOOTER_SIDE_OFFSET;
import static frc.robot.util.Constants.AlphabotLimelightConstants.AlphaSHOOTER_YAW_OFFSET;
import static frc.robot.util.Constants.AlphabotLimelightConstants.BUMP_MINIMUM_THRESHOLD;
import static frc.robot.util.Constants.AlphabotLimelightConstants.FIELD_DIMENSION_X;
import static frc.robot.util.Constants.AlphabotLimelightConstants.FIELD_DIMENSION_Y;
import static frc.robot.util.Constants.AlphabotLimelightConstants.ZERO;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructTopic;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.AlphaSubsystem;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;

public class AlphabotVision extends SubsystemBase {

  private final AlphabotSwerve m_swerve;
  private final Limelight m_bl;
  private final Limelight m_br;
  private final Limelight m_shooter;
  private boolean m_enabled = false;
  private long m_slow_count = 0;

  private NetworkTableInstance nt = NetworkTableInstance.getDefault();
  private StructTopic<Pose2d> m_ashootPose = nt.getStructTopic("/limelight_poses/ashoot", Pose2d.struct);
  private StructTopic<Pose2d> m_mlksrblPose = nt.getStructTopic("/limelight_poses/mlksrbl", Pose2d.struct);
  private StructTopic<Pose2d> m_mlksrbrPose = nt.getStructTopic("/limelight_poses/mlksrbr", Pose2d.struct);
  private StructPublisher<Pose2d> m_ashootPosePublisher;
  private StructPublisher<Pose2d> m_mlksrblPosePublisher;
  private StructPublisher<Pose2d> m_mlksrbrPosePublisher;
  private Pose2d blank = new Pose2d();

  public AlphabotVision() {
    m_swerve = AlphaSubsystem.swerve;
    m_bl = new Limelight(AlphaLL_BL, true);
    m_br = new Limelight(AlphaLL_BR, false);
    m_shooter = new Limelight(AlphaLL_SHOOTER, true);

    limelightSettingConstruction();
    m_ashootPosePublisher = m_ashootPose.publish();
    m_ashootPosePublisher.setDefault(blank);
    m_mlksrblPosePublisher = m_mlksrblPose.publish();
    m_mlksrblPosePublisher.setDefault(blank);
    m_mlksrbrPosePublisher = m_mlksrbrPose.publish();
    m_mlksrbrPosePublisher.setDefault(blank);
    LimelightHelpers.SetIMUMode(AlphaLL_BL, 4);
    LimelightHelpers.SetIMUMode(AlphaLL_SHOOTER, 4);
    LimelightHelpers.SetIMUAssistAlpha(AlphaLL_BL, 0.01);
    LimelightHelpers.SetIMUAssistAlpha(AlphaLL_SHOOTER, 0.01);
  }

  private void limelightSettingConstruction() {
    LimelightHelpers.setLEDMode_ForceOff(m_bl.name);
    LimelightHelpers.setCameraPose_RobotSpace(
        m_bl.name,
        AlphaBL_FORWARD_OFFSET, // Forward offset (meters)
        AlphaBL_SIDE_OFFSET, // Side offset (meters) left is positive
        AlphaBL_HEIGHT_OFFSET, // Height offset (meters)
        AlphaBL_ROLL_OFFSET, // Roll (degrees)
        AlphaBL_PITCH_OFFSET, // Pitch (degrees)
        AlphaBL_YAW_OFFSET // Yaw (degrees)
    );

    LimelightHelpers.setLEDMode_ForceOff(m_br.name);
    LimelightHelpers.setCameraPose_RobotSpace(
        m_br.name,
        AlphaBR_FORWARD_OFFSET, // Forward offset (meters)
        AlphaBR_SIDE_OFFSET, // Side offset (meters) left is positive
        AlphaBR_HEIGHT_OFFSET, // Height offset (meters)
        AlphaBR_ROLL_OFFSET, // Roll (degrees)
        AlphaBR_PITCH_OFFSET, // Pitch (degrees)
        AlphaBR_YAW_OFFSET // Yaw (degrees)
    );

    LimelightHelpers.setLEDMode_ForceOff(m_shooter.name);
    LimelightHelpers.setCameraPose_RobotSpace(
        m_shooter.name,
        AlphaSHOOTER_FORWARD_OFFSET, // Forward offset (meters)
        AlphaSHOOTER_SIDE_OFFSET, // Side offset (meters) left is positive
        AlphaSHOOTER_HEIGHT_OFFSET, // Height offset (meters)
        AlphaSHOOTER_ROLL_OFFSET, // Roll (degrees)
        AlphaSHOOTER_PITCH_OFFSET, // Pitch (degrees)
        AlphaSHOOTER_YAW_OFFSET // Yaw (degrees)
    );
  }

  private void ManageLimelightTemperature() {
    m_slow_count++;
    if (m_enabled && (m_slow_count % 50) != 0) {
      return;
    }
    final int throttle = m_enabled ? 0 : 169;
    m_bl.setThrottle(throttle);
    m_br.setThrottle(throttle);
    m_shooter.setThrottle(throttle);
  }

  public Command AlphabotVisionDefaultCommand() {
    return run(() -> {
    });
  }

  private void setExternalSeed() {
    if (m_enabled) {
      LimelightHelpers.SetIMUMode(AlphaLL_BL, 4);
      LimelightHelpers.SetIMUMode(AlphaLL_SHOOTER, 4);
    } else {
      LimelightHelpers.SetIMUMode(AlphaLL_BL, 1);
      LimelightHelpers.SetIMUMode(AlphaLL_SHOOTER, 1);
    }
  }

  @Override
  public void periodic() {
    m_enabled = DriverStation.isEnabled();
    setExternalSeed();
    ManageLimelightTemperature();

    if (m_swerve == null) {
      return;
    }
    final double yaw_degrees = AlphaSubsystem.swerve.getCurrentPose().getRotation().getDegrees();

    m_shooter.setRobotOrientation(yaw_degrees);
    m_br.setRobotOrientation(yaw_degrees);
    m_bl.setRobotOrientation(yaw_degrees);

    m_shooter.updateFusionOdometry();
    m_br.updateFusionOdometry();
    m_bl.updateFusionOdometry();

    m_shooter.updateYaw();
    m_bl.updateYaw();

    m_ashootPosePublisher.set(m_shooter.getEstimatePoseMT2());
    m_mlksrblPosePublisher.set(m_bl.getEstimatePoseMT2());
    m_mlksrbrPosePublisher.set(m_br.getEstimatePoseMT2());
  }

  @Override
  public void simulationPeriodic() {
  }

  private class Limelight {

    private final String name;
    private final boolean isLL4;
    private double lastFrame = -2;
    private double frame = -2;
    private double BumpScaleFactor = 1;
    private PoseEstimate estimate_MT2;
    private PoseEstimate estimate_MT1;

    Limelight(String p_name, boolean p_isLL4) {
      name = p_name;
      isLL4 = p_isLL4;
    }

    public boolean hasTag() {
      return LimelightHelpers.getTV(name);
    }

    // use external IMU yaw submitted via setRobotOrientation() and configure the
    // LL4 internal IMU's fused yaw to match the submitted yaw value
    // 0 - Use external IMU yaw submitted via SetRobotOrientation() for MT2
    // localization. The internal IMU is ignored entirely.
    // 1 - Use external IMU yaw submitted via SetRobotOrientation(), and configure
    // the LL4 internal IMU's fused yaw to match the submitted yaw value.
    // 2 - Use internal IMU for MT2 localization.
    public void setRobotOrientation(double yawDeg) {
      LimelightHelpers.SetRobotOrientation(name, yawDeg, 0, 0, 0, 0, 0);
    }

    public void updateFusionOdometry() {
      estimate_MT2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
      frame = getFrame();

      if (!verifyPoseValidity()) {
        updateFrame();
        return;
      }

      double numberOfTags = estimate_MT2.tagCount;
      double distance = estimate_MT2.avgTagDist;

      double xystdev = setxystdev(distance, numberOfTags);

      m_swerve.addVisionMeasurement(
          estimate_MT2.pose,
          estimate_MT2.timestampSeconds,
          VecBuilder.fill(xystdev, xystdev, 999999999));

      updateFrame();
    }

    public void updateYaw() {
      frame = getFrame();
      estimate_MT1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);

      if (!verifyYawValidity()) {
        updateFrame();
        return;
      }

      double numberOfTags = estimate_MT1.tagCount;
      double distance = estimate_MT1.avgTagDist;
      double thetastdev = setthetastdev(distance, numberOfTags);
      m_swerve.addVisionMeasurement(estimate_MT1.pose, estimate_MT1.timestampSeconds,
          VecBuilder.fill(99999999, 999999999, thetastdev));
    }

    public Pose2d getEstimatePoseMT2() {
      if (estimate_MT2 == null) {
        return new Pose2d();
      }
      return estimate_MT2.pose;
    }

    public double getEstimateYawMT1() {
      if (estimate_MT1 == null) {
        // value that is unobtainable from a regular pose so we know it's inaccurate
        return 400;
      }
      return estimate_MT1.pose.getRotation().getDegrees();
    }

    /*
     * Verify pose of MT2
     */
    private boolean verifyPoseValidity() {
      return estimate_MT2 != null
          && estimate_MT2.tagCount != 0 // test
          && m_swerve.getState().Speeds.omegaRadiansPerSecond < Math.PI // maybe change to two depending on max speed
          && frame > lastFrame
          && !Double.isNaN(estimate_MT2.avgTagDist)
          && poseInField();
    }

    /*
     * Verify Yaw from MT1
     */
    private boolean verifyYawValidity() {
      return estimate_MT1 != null && estimate_MT1.tagCount > 1
          && m_swerve.getState().Speeds.omegaRadiansPerSecond < Math.PI / 3 && poseInField();
    }

    private void updateFrame() {
      lastFrame = frame;
    }

    private double setxystdev(double distance, double numberOfTags) {
      setBumpScaleFactor();
      double xyStdv = 0;
      double errorFactor = isLL4 ? AlphaERROR_FACTOR_LL4 : AlphaERROR_FACTOR_LL3G;
      double minimumXyStdDev = isLL4 ? AlphaMINIMUM_XY_STD_DEV_LL4 : AlphaMINIMUM_XY_STD_DEV_LL3G;
      xyStdv = Math.max(
          minimumXyStdDev,
          (Math.pow(distance, 3) * errorFactor) * BumpScaleFactor / Math.pow(numberOfTags, 2));
      return xyStdv;
    }

    private double setthetastdev(double distance, double numberOfTags) {
      if (!verifyYawValidity()) {
        return 9999999999.9;
      }
      double errorFactor = isLL4 ? AlphaERROR_FACTOR_LL4 : AlphaERROR_FACTOR_LL3G;
      double thetaStdv = Math.max(1, (Math.pow(distance, 2) * errorFactor));
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
      return Math.abs(m_swerve.getPitch()) > BUMP_MINIMUM_THRESHOLD;
    }

    private void setBumpScaleFactor() {
      if (onBump()) {
        BumpScaleFactor = .5;
      }
      BumpScaleFactor = 1;
    }

    public boolean poseInField() {
      return estimate_MT2.pose.getMeasureX().compareTo(ZERO) >= 0
          && estimate_MT2.pose.getMeasureX().compareTo(FIELD_DIMENSION_X) <= 0
          && estimate_MT2.pose.getMeasureY().compareTo(ZERO) >= 0
          && estimate_MT2.pose.getMeasureY().compareTo(FIELD_DIMENSION_Y) <= 0;
    }
  }
}