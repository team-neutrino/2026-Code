// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.AlphaSubsystem;
import frc.robot.util.DriveToPointPID;

import static frc.robot.util.Constants.DriveToPointConstants.*;

public class DriveToPoint extends Command {
    private DriveToPointPID m_drivePID;
    private Pose2d m_target;

    public DriveToPoint(Pose2d target) {
        addRequirements(AlphaSubsystem.swerve);
        m_drivePID = new DriveToPointPID();
    }

    private void drive() {
        double xVelocity = m_drivePID.getXVelocity(), yVelocity = m_drivePID.getYVelocity();

        xVelocity = MathUtil.clamp(xVelocity, -MAX_DRIVETOPOINT_SPEED, MAX_DRIVETOPOINT_SPEED);
        yVelocity = MathUtil.clamp(yVelocity, -MAX_DRIVETOPOINT_SPEED, MAX_DRIVETOPOINT_SPEED);

        AlphaSubsystem.swerve.setVelocity(xVelocity, yVelocity, m_drivePID.getRotation());
    }

    @Override
    public void initialize() {
        m_drivePID.setTarget(m_target);
    }

    @Override
    public void execute() {
        drive();
    }

    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
