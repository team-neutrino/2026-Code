// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.util.Constants.DriveToPointConstants.*;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;

import static frc.robot.util.Constants.GlobalConstants.RED_ALLIANCE;
import static frc.robot.util.AlphaSubsystem.*;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.util.AlphaSubsystem;

public class SplineToPoint extends Command {
  private CommandXboxController m_driverController;
  private List<Pose2d> m_targetPoseList;
  private Pose2d m_target;
  private boolean m_hadNoFuel;
  private boolean m_wasActive;
  private TargetMode m_targetMode;
  NetworkTableInstance nt = NetworkTableInstance.getDefault();
  private final NetworkTable driveStateTable = nt.getTable("DriveToPoint");
  private final StructPublisher<Pose2d> driveTarget = driveStateTable.getStructTopic("TargetPose", Pose2d.struct)
      .publish();

  public SplineToPoint(CommandXboxController driverController, TargetMode targetMode) {
    m_driverController = driverController;
    m_targetMode = targetMode;
  }

  private Pose2d getClosestPoint(List<Pose2d> list) {
    return swerve.getCurrentPose().nearest(list);
  }

  private boolean isHopperEmpty() {
    return alphaIntake.isEmpty();
  }

  private void spline(Pose2d target) {
    PathConstraints constraints = new PathConstraints(SPLINE_MAX_SPEED, SPLINE_MAX_ACCELERATION,
        SPLINE_MAX_ANGULAR_VELOCITY, SPLINE_MAX_ANGULAR_ACCELERATION);
    Command pathCommand = AutoBuilder.pathfindToPose(target, constraints, SPLINE_END_VELOCITY);
    switch (m_targetMode) {
      case SHOOTING:
        CommandScheduler.getInstance().schedule(
            pathCommand.until(() -> (Math.abs(AlphaSubsystem.swerve.getCurrentPose().getX() - m_target.getX()) < 1
                && Math.abs(AlphaSubsystem.swerve.getCurrentPose().getY() - m_target.getY()) < 1))
                .andThen(new DriveToPoint(target)).until(() -> !m_driverController.getHID().getAButton()));
      case CLIMBING:
        CommandScheduler.getInstance().schedule(
            pathCommand.until(() -> (Math.abs(AlphaSubsystem.swerve.getCurrentPose().getX() - m_target.getX()) < 1
                && Math.abs(AlphaSubsystem.swerve.getCurrentPose().getY() - m_target.getY()) < 1))
                .andThen(new DriveToPoint(target)
                    .until(() -> (Math.abs(AlphaSubsystem.swerve.getCurrentPose().getX() - m_target.getX()) < 1
                        && Math.abs(AlphaSubsystem.swerve.getCurrentPose().getY() - m_target.getY()) < 0.1)))
                .andThen(new AlignToClimb()));
    }

  }

  private void setTarget() {
    switch (m_targetMode) {
      case SHOOTING:
        if (RED_ALLIANCE.get()) {
          if (!isHopperEmpty()) {
            m_targetPoseList = RED_RADIAL_SHOOTING_POSES;
            m_target = getClosestPoint(m_targetPoseList);
          } else if (isHopperEmpty() && !hubState.isRedHubActive()) {
            m_targetPoseList = RED_NEUTRAL_ZONE_POSES;
            m_target = getClosestPoint(m_targetPoseList);
          } else if (isHopperEmpty() && hubState.isRedHubActive()) {
            // Do nothing
          }
        } else {
          if (!isHopperEmpty()) {
            m_targetPoseList = BLUE_RADIAL_SHOOTING_POSES;
            m_target = getClosestPoint(m_targetPoseList);
          } else if (isHopperEmpty() && !hubState.isRedHubActive()) {
            m_targetPoseList = BLUE_NEUTRAL_ZONE_POSES;
            m_target = getClosestPoint(m_targetPoseList);
          } else if (isHopperEmpty() && hubState.isRedHubActive()) {
            // Do nothing
          }
        }
      case SHUTTLING:
        if (RED_ALLIANCE.get()) {
          m_targetPoseList = RED_SHUTTLE_POSES;
          m_target = getClosestPoint(m_targetPoseList);
        } else {
          m_targetPoseList = BLUE_SHUTTLE_POSES;
          m_target = getClosestPoint(m_targetPoseList);
        }
      case CLIMBING:
        if (RED_ALLIANCE.get()) {
          m_targetPoseList = RED_CLIMB_POSES;
          m_target = getClosestPoint(m_targetPoseList);
        } else {
          m_targetPoseList = BLUE_CLIMB_POSES;
          m_target = getClosestPoint(m_targetPoseList);
        }
    }
  }

  @Override
  public void initialize() {
    m_hadNoFuel = isHopperEmpty();
    m_wasActive = RED_ALLIANCE.get() ? hubState.isRedHubActive() : hubState.isBlueHubActive();
    setTarget();
    final long now = NetworkTablesJNI.now();
    driveTarget.set(m_target, now);
    spline(m_target);
  }

  @Override
  public void execute() {
    // logic to re-initialize if we use "bumpers" or equivalent
    if (isHopperEmpty() != m_hadNoFuel || RED_ALLIANCE.get() ? hubState.isRedHubActive()
        : hubState.isBlueHubActive() != m_wasActive) {
      initialize();
    } // re-initialize if the state of our hopper or hub changes
    final long now = NetworkTablesJNI.now();
    driveTarget.set(m_target, now);
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
