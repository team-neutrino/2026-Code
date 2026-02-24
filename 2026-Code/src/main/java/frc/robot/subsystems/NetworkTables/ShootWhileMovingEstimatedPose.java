package frc.robot.subsystems.NetworkTables;

import static frc.robot.util.Constants.ClimbConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructTopic;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Swerve;
import frc.robot.util.PIDTuner;

public class ShootWhileMovingEstimatedPose extends Swerve {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    private Pose2d blank = new Pose2d();

    private StructTopic<Pose2d> estimatedPose = nt.getStructTopic("/limelight_poses/front", Pose2d.struct);
    private StructPublisher<Pose2d> m_estimatedPosePub;

    public ShootWhileMovingEstimatedPose() {
        m_estimatedPosePub = estimatedPose.publish();
        m_estimatedPosePub.setDefault(blank);

    }

    @Override
    public void periodic() {
        super.periodic();
        final long now = NetworkTablesJNI.now();

        m_estimatedPosePub.set(getProjectedPose(), now);

    }
}
