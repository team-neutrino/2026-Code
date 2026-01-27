// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.alpha_subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;
import frc.robot.util.AlphaSubsystem;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructTopic;

import static edu.wpi.first.units.Units.Meter;
import static frc.robot.util.Constants.AlphabotLimelightConstants.*;

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
    m_enabled = DriverStation.isEnabled();
    final int throttle = m_enabled ? 0 : 169;
    m_bl.setThrottle(throttle);
    m_br.setThrottle(throttle);
    m_shooter.setThrottle(throttle);
  }

  public Command AlphabotVisionDefaultCommand() {
    return run(() -> {
    });
  }

  @Override
  public void periodic() {
    ManageLimelightTemperature();

    if (m_swerve == null) {
      return;
    }

    final double yaw_degrees = m_swerve.getYawDegrees();

    m_shooter.setRobotOrientation(yaw_degrees);
    m_br.setRobotOrientation(yaw_degrees);
    m_bl.setRobotOrientation(yaw_degrees);

    m_shooter.updateFusionOdometry();
    m_br.updateFusionOdometry();
    m_bl.updateFusionOdometry();

    m_ashootPosePublisher.set(m_shooter.getEstimatePose());
    m_mlksrblPosePublisher.set(m_bl.getEstimatePose());
    m_mlksrbrPosePublisher.set(m_br.getEstimatePose());
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
    private PoseEstimate estimate;
    private static final Distance ZERO = Distance.ofBaseUnits(0, Meter);
    private static final Distance FIELD_DIMENSION_X = Distance.ofBaseUnits(Units.inchesToMeters(650.12), Meter);
    private static final Distance FIELD_DIMENSION_Y = Distance.ofBaseUnits(Units.inchesToMeters(316.64), Meter);
    private static final double BUMP_MINIMUM_THRESHOLD = 7;

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
      estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
      frame = getFrame();

      if (!verifyLimelightValidity()) {
        updateFrame();
        return;
      }

      double numberOfTags = estimate.tagCount;
      double distance = estimate.avgTagDist;

      double xystdev = setxystdev(distance, numberOfTags);
      double thetastdev = setthetastdev(distance, numberOfTags);

      m_swerve.addVisionMeasurement(
          estimate.pose,
          estimate.timestampSeconds,
          VecBuilder.fill(xystdev, xystdev, thetastdev));

      updateFrame();
    }

    public Pose2d getEstimatePose() {
      if (estimate == null) {
        return new Pose2d();
      }
      return estimate.pose;
    }

    private boolean verifyLimelightValidity() {
      return estimate != null
          && estimate.tagCount != 0 // test
          && m_swerve.getState().Speeds.omegaRadiansPerSecond < Math.PI // maybe change to two depending on max speed
          && frame > lastFrame
          && !Double.isNaN(estimate.avgTagDist)
          && poseInField();
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
      double thetaStdv = 999999999;
      // double errorFactor = isLL4 ? AlphaERROR_FACTOR_LL4_ANGLE :
      // AlphaERROR_FACTOR_LL3G_ANGLE;
      // double minimumThetaStdDev = isLL4 ? AlphaMINIMUM_THETA_STD_DEV_LL4 :
      // AlphaMINIMUM_THETA_STD_DEV_LL3G;
      // thetaStdv = Math.max(
      // minimumThetaStdDev,
      // (distance * errorFactor) / numberOfTags);
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
      return estimate.pose.getMeasureX().compareTo(ZERO) >= 0
          && estimate.pose.getMeasureX().compareTo(FIELD_DIMENSION_X) <= 0
          && estimate.pose.getMeasureY().compareTo(ZERO) >= 0
          && estimate.pose.getMeasureY().compareTo(FIELD_DIMENSION_Y) <= 0;
    }
  }
}