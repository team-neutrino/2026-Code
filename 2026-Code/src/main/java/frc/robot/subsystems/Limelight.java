// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;
import frc.robot.util.Subsystem;
import frc.robot.util.Constants;
import static frc.robot.util.Constants.LimelightConstants.*;

  //Uncommment everything with swerve in it when swerve is added

public class Limelight extends SubsystemBase {
  LimelightHelpers m_limelightHelpers;
  double m_robotYaw;
  // Swerve m_swerve;
  Rotation2d m_targetYaw;
  private double m_lastFrameFr = -2;
  private double m_lastFrameFl = -2;
  private double m_lastFrameBr = -2;
  private double m_lastFrameBl = -2;
  private double m_lastFrameShooter = -2;
  private boolean m_has_shooter_tag;
  private boolean m_has_fr_tag;
  private boolean m_has_fl_tag;
  private boolean m_has_br_tag;
  private boolean m_has_bl_tag;
  private boolean m_enabled = false;
  private long m_slow_count = 0;

  public Limelight() {
    // m_swerve = Subsystem.swerve;
    m_limelightHelpers = new LimelightHelpers();
    // fake pipeline number
    // LimelightHelpers.setPipelineIndex(LIMELIGHT_1, 1);
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

    LimelightHelpers.SetFiducialDownscalingOverride(LL_FR, 3);
    LimelightHelpers.setLEDMode_ForceOff(LL_SHOOTER);
    LimelightHelpers.setCameraPose_RobotSpace(LL_SHOOTER,
        SHOOTER_FORWARD_OFFSET, // Forward offset (meters)
        SHOOTER_SIDE_OFFSET, // Side offset (meters) left is positive
        SHOOTER_HEIGHT_OFFSET, // Height offset (meters)
        SHOOTER_ROLL_OFFSET, // Roll (degrees)
        SHOOTER_PITCH_OFFSET, // Pitch (degrees)
        SHOOTER_YAW_OFFSET // Yaw (degrees)
    );

    LimelightHelpers.SetFiducialDownscalingOverride(LL_SHOOTER, 3);

    LimelightHelpers.SetIMUMode(LL_SHOOTER, 1);
    LimelightHelpers.SetIMUMode(LL_FR, 1);
    // use external IMU yaw submitted via setRobotOrientation() and configure the
    // LL4 internal IMU's fused yaw to match the submitted yaw value
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

  private boolean verifyLimelightValidity(PoseEstimate estimate, double frame){
    return (estimate != null && estimate.tagCount != 0
        // && m_swerve.getState().Speeds.omegaRadiansPerSecond < 4 * Math.PI
        && frame > m_lastFrameShooter);
  }

  private void updateFrame(PoseEstimate x, double frame, String name){
      switch(name){
        case LL_FR:
          m_lastFrameFr = frame;
          break;
        case LL_FL:
          m_lastFrameFl = frame;
          break;
        case LL_BR:
          m_lastFrameBr = frame;
          break;
        case LL_BL:
          m_lastFrameBl = frame;
          break;
        case LL_SHOOTER:
          m_lastFrameShooter = frame;
          break;
      }
  }

  private double setxytdev(double distance, double numberOfTags, String name){
    double xyStdv;
    if(name.equals(LL_BL)){
      xyStdv = Math.max(Minimum_XY_Std_Dev_LL3,(distance*ErrorFactor_LL3)/numberOfTags);
    }
    else if(name.equals(LL_BR)){
      xyStdv = Math.max(Minimum_XY_Std_Dev_LL2,(distance*ErrorFactor_LL2)/numberOfTags);
    }
    else if(name.equals(LL_FL)){
      xyStdv = Math.max(Minimum_XY_Std_Dev_LL3g,(distance*ErrorFactor_LL3g)/numberOfTags);
    }
    else{
      xyStdv = Math.max(Minimum_XY_Std_Dev_LL4,(distance*ErrorFactor_LL4)/numberOfTags);
    }
  }

  private double setthetastdev(double distance, double numberOfTags, String name){
    double thetaStdv;
    if(name.equals(LL_BL)){
      thetaStdv = Math.max(Minimum_Theta_Std_Dev_LL3,(distance*ErrorFactor_LL3_Angle)/numberOfTags);
    }
    else if(name.equals(LL_BR)){
      thetaStdv = Math.max(Minimum_Theta_Std_Dev_LL2,(distance*ErrorFactor_LL2_Angle)/numberOfTags);
    }
    else if(name.equals(LL_FL)){
      thetaStdv = Math.max(Minimum_Theta_Std_Dev_LL3g,(distance*ErrorFactor_LL3g_Angle)/numberOfTags);
    }
    else{
      thetaStdv = Math.max(Minimum_Theta_Std_Dev_LL4,(distance*ErrorFactor_LL4_Angle)/numberOfTags);
    }
    return thetaStdv;
  }

  
  private void updateFusionOdometry() {
    // m_swerve.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
    LimelightHelpers.PoseEstimate estimateBL = LimelightHelpers
        .getBotPoseEstimate_wpiBlue_MegaTag2(LL_BL);
    LimelightHelpers.PoseEstimate estimateBR = LimelightHelpers
        .getBotPoseEstimate_wpiBlue_MegaTag2(LL_BR);
    LimelightHelpers.PoseEstimate estimateFL = LimelightHelpers
        .getBotPoseEstimate_wpiBlue_MegaTag2(LL_FL);
    LimelightHelpers.PoseEstimate estimateFR = LimelightHelpers
        .getBotPoseEstimate_wpiBlue_MegaTag2(LL_FR);

    record PoseData(PoseEstimate estimate, double frame, String limelightId) {}

    PoseData BL = new PoseData(estimateBL, getFrame(LL_BL), LL_BL);
    PoseData BR = new PoseData(estimateBR, getFrame(LL_BR), LL_BR);
    PoseData FL = new PoseData(estimateFL, getFrame(LL_FL),LL_FL);
    PoseData FR = new PoseData(estimateFR, getFrame(LL_FR), LL_FR);
    PoseData[] limelights = {BL, BR, FL, FR};

    for(PoseData limelight:limelights) {
      updateFrame(limelight.estimate(), getFrame(limelight.limelightId()),limelight.limelightId());
      if(!verifyLimelightValidity(limelight.estimate(), getFrame(limelight.limelightId()))){
        continue;
      }
      double numberOfTags = limelight.estimate().tagCount;
      double distance = limelight.estimate().avgTagDist;
      double xystdev = setxytdev(distance, numberOfTags, limelight.limelightId());
      double thetastdev = setthetastdev(distance, numberOfTags, limelight.limelightId());

    //check 1st and 2nd argument
      // m_swerve.addVisionMeasurement(limelight.estimate().pose, limelight.estimate().timestampSeconds,VecBuilder.fill(xystdev, xystdev, thetastdev));
    }
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

  public double getTargetYawFromShooter() {
    double[] temp = LimelightHelpers.getTargetPose_RobotSpace(LL_SHOOTER);
    return temp.length == 0 ? 0 : temp[4];
  }


  private double getFrame(String limelight) {
    return NetworkTableInstance.getDefault().getTable(limelight).getEntry("hb").getDouble(-1);
  }

  //find alternative to this function during testing. setThrottle no longer exists
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
  // LimelightHelpers.SetThrottle(LimelightConstants.LL_FR, throttle);
  // LimelightHelpers.SetThrottle(LimelightConstants.LL_FL, throttle);
  // LimelightHelpers.SetThrottle(LimelightConstants.LL_BR, throttle);
  // LimelightHelpers.SetThrottle(LimelightConstants.LL_BL, throttle);
  }

  public Command limelightDefaultCommand() {
    return run(() -> {

    });
  }

  @Override
  public void periodic() {
    ManageLimelightTemperature();
  // read the five Limelight tables and store whether each currently sees a target
  m_has_shooter_tag = LimelightHelpers.getTV(LL_SHOOTER);
  m_has_fr_tag = LimelightHelpers.getTV(LL_FR);
  m_has_fl_tag = LimelightHelpers.getTV(LL_FL);
  m_has_br_tag = LimelightHelpers.getTV(LL_BR);
  m_has_bl_tag = LimelightHelpers.getTV(LL_BL);

    //changed - when enabled configure IMU mode for all Limelights we use
    if (m_enabled) {
      LimelightHelpers.SetIMUMode(LL_SHOOTER, 2);
      LimelightHelpers.SetIMUMode(LL_FR, 2);
      LimelightHelpers.SetIMUMode(LL_FL, 2);
      LimelightHelpers.SetIMUMode(LL_BR, 2);
      LimelightHelpers.SetIMUMode(LL_BL, 2);
    }

    // if (m_swerve == null) {
    //   return;
    // }

  //dummy value until swerve is added
  // final var yaw_degrees = Subsystem.swerve.getYawDegrees();
  final var yaw_degrees = 0;


    // according to limelight docs, this needs to be called before using
    // .getBotPoseEstimate_wpiBlue_MegaTag2
  // supply current robot orientation to every Limelight before asking for pose estimates
  LimelightHelpers.SetRobotOrientation(LL_SHOOTER, yaw_degrees, 0, 0, 0, 0, 0);
  LimelightHelpers.SetRobotOrientation(LL_FR, yaw_degrees, 0, 0, 0, 0, 0);
  LimelightHelpers.SetRobotOrientation(LL_FL, yaw_degrees, 0, 0, 0, 0, 0);
  LimelightHelpers.SetRobotOrientation(LL_BR, yaw_degrees, 0, 0, 0, 0, 0);
  LimelightHelpers.SetRobotOrientation(LL_BL, yaw_degrees, 0, 0, 0, 0, 0);
    updateFusionOdometry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}