// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.util.Constants.DriveToPointConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.util.AlphaSubsystem;

public class SplineToPoint extends PointControl {
  private CommandXboxController m_driverController;

  public SplineToPoint(CommandXboxController driverController) {
    super();
    m_driverController = driverController;
  }

  private void spline(Pose2d target) {
    PathConstraints constraints = new PathConstraints(SPLINE_MAX_SPEED, SPLINE_MAX_ACCELERATION,
        SPLINE_MAX_ANGULAR_VELOCITY, SPLINE_MAX_ANGULAR_ACCELERATION);
    Command pathCommand = AutoBuilder.pathfindToPose(target, constraints, SPLINE_END_VELOCITY);
    CommandScheduler.getInstance().schedule(
        pathCommand.until(() -> (Math.abs(AlphaSubsystem.swerve.getCurrentPose().getX() - getTarget().getX()) < 1
            && Math.abs(AlphaSubsystem.swerve.getCurrentPose().getY() - getTarget().getY()) < 1))
            .andThen(new DriveToPoint()).until(() -> !m_driverController.getHID().getAButton()));
  }

  @Override
  public void initialize() {
    super.initialize();
    spline(getTarget());
  }

  @Override
  public void execute() {
    super.execute();
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
