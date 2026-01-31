package frc.robot.subsystems.NetworkTables;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import frc.robot.subsystems.Climb;

public class ClimbNT extends Climb {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();

    DoubleTopic actualClimbPosition = nt.getDoubleTopic("/climb/actual_climb_position");
    DoubleTopic targetClimbPosition = nt.getDoubleTopic("/climb/target_climb_position");
    BooleanTopic atTargetClimbPosition = nt.getBooleanTopic("/climb/at_target_climb_position");
    DoubleTopic CANRangeDistance = nt.getDoubleTopic("/climb/climb_CANRange_distance");
    BooleanTopic CANRangeDetection = nt.getBooleanTopic("/climb/climb_CANRange_detection");
    BooleanTopic CANAndColorDetection = nt.getBooleanTopic("/climb/climb_CANandColor_detection");

    final DoublePublisher actualClimbPositionPub;
    final DoublePublisher targetClimbPositionPub;
    final BooleanPublisher atTargetClimbPositionPub;
    final DoublePublisher CANRangeDistancePub;
    final BooleanPublisher CANRangeDetectionPub;
    final BooleanPublisher CANAndColorDetectionPub;

    public ClimbNT() {
        actualClimbPositionPub = actualClimbPosition.publish();
        actualClimbPositionPub.setDefault(0.0);

        targetClimbPositionPub = targetClimbPosition.publish();
        targetClimbPositionPub.setDefault(0.0);

        atTargetClimbPositionPub = atTargetClimbPosition.publish();
        atTargetClimbPositionPub.setDefault(false);

        CANRangeDistancePub = CANRangeDistance.publish();
        CANRangeDistancePub.setDefault(0.0);

        CANRangeDetectionPub = CANRangeDetection.publish();
        CANRangeDetectionPub.setDefault(false);

        CANAndColorDetectionPub = CANAndColorDetection.publish();
        CANAndColorDetectionPub.setDefault(false);
    }

    @Override
    public void periodic() {
        super.periodic();
        final long now = NetworkTablesJNI.now();

        actualClimbPositionPub.set(getClimbPosition(), now);
        targetClimbPositionPub.set(getClimbTargetPosition(), now);
        atTargetClimbPositionPub.set(atTargetPosition(), now);
        CANRangeDistancePub.set(getCANRangeDistance(), now);
        CANRangeDetectionPub.set(isCANRangeDetected(), now);
        CANAndColorDetectionPub.set(isClimbOverBar(), now);
    }
}
