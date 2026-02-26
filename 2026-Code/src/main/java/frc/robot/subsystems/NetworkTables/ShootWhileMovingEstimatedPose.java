package frc.robot.subsystems.NetworkTables;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructTopic;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Swerve;

public class ShootWhileMovingEstimatedPose extends Swerve {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    private Pose2d blank = new Pose2d();

    private StructTopic<Pose2d> estimatedPose = nt.getStructTopic("/EstimatedPose", Pose2d.struct);
    private StructPublisher<Pose2d> m_estimatedPosePub;

    public ShootWhileMovingEstimatedPose() {
        m_estimatedPosePub = estimatedPose.publish();
        m_estimatedPosePub.setDefault(blank);

    }

    @Override
    public void periodic() {
        super.periodic();
        final long now = NetworkTablesJNI.now();

        m_estimatedPosePub.set(getHubPose2(), now);

    }
}
