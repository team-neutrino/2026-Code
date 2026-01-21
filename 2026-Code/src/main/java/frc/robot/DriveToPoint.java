// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.Constants.GlobalConstants;
import frc.robot.util.Subsystem;

import static frc.robot.util.Subsystem.*;

public class DriveToPoint extends Command {
  private List<Pose2d> m_targetPoseList;
  private Pose2d m_target;

  public DriveToPoint() {
    addRequirements(swerve);
  }

  public void spline(Pose2d target) {
    PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI);
    Command pathCommand = AutoBuilder.pathfindToPose(target, constraints);
    pathCommand.schedule();
  }

  private

      void setTarget() {
    // logic for selecting target

  }

  @Override
  public void initialize() {
    setTarget();
    // initialize list of points to go through
    // set driving to point true
  }

  @Override
  public void execute() {
    // logic to re-initialize if the state of our hopper changes between empty and
    // full
    // logic to re-initialize if we use "bumpers" or equivalent
    // actually drive
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
