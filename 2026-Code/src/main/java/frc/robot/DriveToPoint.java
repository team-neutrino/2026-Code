// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.Subsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToPoint extends Command {
  /** Creates a new DriveToPoint. */
  public DriveToPoint() {
    addRequirements(Subsystem.swerve);
  }

  private void setTarget() {

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
