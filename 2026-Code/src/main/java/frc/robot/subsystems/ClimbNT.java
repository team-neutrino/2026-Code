package frc.robot.subsystems;

import static frc.robot.util.Constants.ClimbConstants.*;

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
    DoubleTopic climbCurrent = nt.getDoubleTopic("/climb/climb_current");
    DoubleTopic CANRangeDistance = nt.getDoubleTopic("/climb/climb_CANRange_distance");
    BooleanTopic CANRangeDetection = nt.getBooleanTopic("/climb/climb_CANRange_detection");
    BooleanTopic CANAndColorDetection = nt.getBooleanTopic("/climb/climb_CANandColor_detection");

    final DoublePublisher actualClimbPositionPub;
    final DoublePublisher targetClimbPositionPub;
    final DoublePublisher climbCurrentPub;
    final DoublePublisher CANRangeDistancePub;
    final BooleanPublisher CANRangeDetectionPub;
    final BooleanPublisher CANAndColorDetectionPub;

    private double m_previousClimbP = CLIMB_kP;
    private double m_previousClimbI = CLIMB_kI;
    private double m_previousClimbD = CLIMB_kD;

    public ClimbNT() {
        actualClimbPositionPub = actualClimbPosition.publish();
        actualClimbPositionPub.setDefault(0.0);

        targetClimbPositionPub = targetClimbPosition.publish();
        targetClimbPositionPub.setDefault(0.0);

        climbCurrentPub = climbCurrent.publish();
        climbCurrentPub.setDefault(0.0);

        CANRangeDistancePub = CANRangeDistance.publish();
        CANRangeDistancePub.setDefault(0.0);

        CANRangeDetectionPub = CANRangeDetection.publish();
        CANRangeDetectionPub.setDefault(false);

        CANAndColorDetectionPub = CANAndColorDetection.publish();
        CANAndColorDetectionPub.setDefault(false);

        // m_PIDTuner = new PIDTuner("climb/{tuning}PID");
        // m_PIDTuner.setP(m_previousClimbP);
        // m_PIDTuner.setI(m_previousClimbI);
        // m_PIDTuner.setD(m_previousClimbD);
    }

    @Override
    public void periodic() {
        super.periodic();
        final long now = NetworkTablesJNI.now();

        actualClimbPositionPub.set(getClimbPosition(), now);
        targetClimbPositionPub.set(getClimbTargetPosition(), now);
        climbCurrentPub.set(getClimbCurrent(), now);
        CANRangeDistancePub.set(getDistance(), now);
        CANRangeDetectionPub.set(isClimbOverBar(), now); // Change later
        CANAndColorDetectionPub.set(isClimbOverBar(), now);

        // if (m_PIDTuner.isDifferentValues(m_previousClimbP, m_previousClimbI,
        // m_previousClimbD)) {
        // changePID(m_PIDTuner.getP(), m_PIDTuner.getI(), m_PIDTuner.getD());
        // m_previousClimbP = m_PIDTuner.getP();
        // m_previousClimbI = m_PIDTuner.getI();
        // m_previousClimbD = m_PIDTuner.getD();
        // }
    }
}
