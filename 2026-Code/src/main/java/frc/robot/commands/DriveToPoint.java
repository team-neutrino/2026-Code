// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.util.Subsystems.swerve;
import frc.robot.util.DriveToPointPID;


import static frc.robot.util.Constants.DriveToPointConstants.*;

import java.util.List;

public class DriveToPoint extends Command {
    private DriveToPointPID m_drivePID;
    private Pose2d m_target;
    private List<Pose2d> m_poseList;
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
  private final NetworkTable driveStateTable = nt.getTable("DriveToPoint");
  private final StructPublisher<Pose2d> driveTarget = driveStateTable.getStructTopic("TargetPose", Pose2d.struct)
      .publish();


    public DriveToPoint(List<Pose2d> shootPoses) {

        addRequirements(swerve);
        m_poseList = shootPoses;
        m_drivePID = new DriveToPointPID();
        }

    private void drive() {
        double xVelocity = m_drivePID.getXVelocity(), yVelocity = m_drivePID.getYVelocity();

        xVelocity = MathUtil.clamp(xVelocity, -MAX_DRIVETOPOINT_SPEED, MAX_DRIVETOPOINT_SPEED);
        yVelocity = MathUtil.clamp(yVelocity, -MAX_DRIVETOPOINT_SPEED, MAX_DRIVETOPOINT_SPEED);

        swerve.setVelocity(xVelocity, yVelocity, m_drivePID.getRotation());
    }

    public Pose2d getTarget() {
        return m_target;
    }

    @Override
    public void initialize() {
        m_target = swerve.getCurrentPose().nearest(m_poseList);
        m_drivePID.setTarget(m_target);
        
    }

    @Override
    public void execute() {
        drive();
        final long now = NetworkTablesJNI.now();
        driveTarget.set(m_target, now);
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
