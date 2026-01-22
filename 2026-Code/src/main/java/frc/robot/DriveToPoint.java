// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

import static frc.robot.util.Constants.DriveToPointConstants.*;
import static frc.robot.util.Constants.FieldMeasurementConstants.*;
import static frc.robot.util.Constants.GlobalConstants.RED_ALLIANCE;
import static frc.robot.util.AlphaSubsystem.*;

public class DriveToPoint extends Command {
  private List<Pose2d> m_targetPoseList;
  private Pose2d m_target;
  private boolean m_hadNoFuel;

  public DriveToPoint() {
    addRequirements(swerve);
  }

  private void spline(Pose2d target) {
    PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI);
    Command pathCommand = AutoBuilder.pathfindToPose(target, constraints);
    pathCommand.schedule();
  }

  private double getCurrentPoseX() {
    return swerve.getCurrentPose().getMeasureX().baseUnitMagnitude();
  }

  private Pose2d getClosestPoint(List<Pose2d> list) {
    return swerve.getCurrentPose().nearest(list);
  }

  private boolean isHopperEmpty() {
    return alphaIntake.isEmpty();
  }

  private void setTarget() {
    // Red Alliance Cases
    if (RED_ALLIANCE.get()) {
      if (getCurrentPoseX() >= ALLIANCE_ZONE_RED && !isHopperEmpty()) {
        m_targetPoseList = RED_RADIAL_SHOOTING_POSES;
        m_target = getClosestPoint(m_targetPoseList);
      } else if (getCurrentPoseX() < ALLIANCE_ZONE_RED && getCurrentPoseX() > ALLIANCE_ZONE_BLUE
          && !isHopperEmpty() && hubState.isRedHubActive()) {
        m_targetPoseList = RED_RADIAL_SHOOTING_POSES;
        m_target = getClosestPoint(m_targetPoseList);
      } else if (getCurrentPoseX() >= ALLIANCE_ZONE_RED && isHopperEmpty()) {
        m_targetPoseList = RED_NEUTRAL_ZONE_POSES;
        m_target = getClosestPoint(m_targetPoseList);
      } else if (getCurrentPoseX() < ALLIANCE_ZONE_RED && getCurrentPoseX() > ALLIANCE_ZONE_BLUE
          && isHopperEmpty()) {
        m_targetPoseList = RED_CLIMB_POSES;
        m_target = getClosestPoint(m_targetPoseList);
      } else if (getCurrentPoseX() < ALLIANCE_ZONE_RED && getCurrentPoseX() > ALLIANCE_ZONE_BLUE
          && !isHopperEmpty() && !hubState.isRedHubActive()) {
        m_targetPoseList = RED_SHUTTLE_POSES;
        m_target = getClosestPoint(m_targetPoseList);
      }

    }
    // Blue Alliance Cases
    else {
      if (getCurrentPoseX() <= ALLIANCE_ZONE_BLUE && !isHopperEmpty()) {
        m_targetPoseList = BLUE_RADIAL_SHOOTING_POSES;
        m_target = getClosestPoint(m_targetPoseList);
      } else if (getCurrentPoseX() < ALLIANCE_ZONE_RED && getCurrentPoseX() > ALLIANCE_ZONE_BLUE
          && !isHopperEmpty() && hubState.isBlueHubActive()) {
        m_targetPoseList = BLUE_RADIAL_SHOOTING_POSES;
        m_target = getClosestPoint(m_targetPoseList);
      } else if (getCurrentPoseX() <= ALLIANCE_ZONE_BLUE && isHopperEmpty()) {
        m_targetPoseList = BLUE_NEUTRAL_ZONE_POSES;
        m_target = getClosestPoint(m_targetPoseList);
      } else if (getCurrentPoseX() < ALLIANCE_ZONE_RED && getCurrentPoseX() > ALLIANCE_ZONE_BLUE
          && isHopperEmpty()) {
        m_targetPoseList = BLUE_CLIMB_POSES;
        m_target = getClosestPoint(m_targetPoseList);
      } else if (getCurrentPoseX() < ALLIANCE_ZONE_RED && getCurrentPoseX() > ALLIANCE_ZONE_BLUE
          && !isHopperEmpty() && !hubState.isBlueHubActive()) {
        m_targetPoseList = BLUE_SHUTTLE_POSES;
        m_target = getClosestPoint(m_targetPoseList);
      }
    }
  }

  @Override
  public void initialize() {
    m_hadNoFuel = isHopperEmpty();
    setTarget();
  }

  @Override
  public void execute() {
    // logic to re-initialize if we use "bumpers" or equivalent
    // actually drive
    if (isHopperEmpty() != m_hadNoFuel) {
      initialize();
    }
    spline(m_target);
  }

  @Override
  public void end(boolean interrupted) {
    // set driving to point false
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
