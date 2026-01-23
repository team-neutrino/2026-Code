package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import static frc.robot.util.Constants.DriveToPointConstants.*;

public class DriveToPointPID {
    private PIDController m_xControl = new PIDController(DRIVE_TO_POINT_P, DRIVE_TO_POINT_I, DRIVE_TO_POINT_D);
    private PIDController m_yControl = new PIDController(DRIVE_TO_POINT_P, DRIVE_TO_POINT_I, DRIVE_TO_POINT_D);
    private static Pose2d m_target = new Pose2d(0, 0, new Rotation2d());

    public void setTarget(Pose2d target) {
        m_target = target;
    }

    public double getXDistance() {
        return Math.abs(m_target.getX() - AlphaSubsystem.swerve.getCurrentPose().getX());
    }

    public double getYDistance() {
        return Math.abs(m_target.getY() - AlphaSubsystem.swerve.getCurrentPose().getY());
    }

    public double getYVelocity() {
        return m_yControl.calculate(AlphaSubsystem.swerve.getCurrentPose().getY() - m_target.getY());
    }

    public double getXVelocity() {
        return m_xControl.calculate(AlphaSubsystem.swerve.getCurrentPose().getX() - m_target.getX());
    }

    public Rotation2d getRotation() {
        return Rotation2d.fromDegrees(m_target.getRotation().getDegrees());
    }

}
