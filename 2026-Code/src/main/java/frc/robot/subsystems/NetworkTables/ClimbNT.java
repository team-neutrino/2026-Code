package frc.robot.subsystems.NetworkTables;

import static frc.robot.util.Constants.ClimbConstants.*;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import frc.robot.subsystems.Climb;
import frc.robot.util.PIDTuner;

public class ClimbNT extends Climb {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();

    DoubleTopic actualClimbPosition = nt.getDoubleTopic("/climb/actual_climb_position");
    DoubleTopic targetClimbPosition = nt.getDoubleTopic("/climb/target_climb_position");
    BooleanTopic atTargetClimbPosition = nt.getBooleanTopic("/climb/at_target_climb_position");
    DoubleTopic CANRangeDistance = nt.getDoubleTopic("/climb/climb_CANRange_distance");
    BooleanTopic CANRangeDetection = nt.getBooleanTopic("/climb/climb_CANRange_detection");

    final DoublePublisher actualClimbPositionPub;
    final DoublePublisher targetClimbPositionPub;
    final BooleanPublisher atTargetClimbPositionPub;
    final DoublePublisher CANRangeDistancePub;
    final BooleanPublisher CANRangeDetectionPub;

    private PIDTuner m_climbPIDTuner;

    private long m_previousClimbSlot;
    private double m_previousClimbKP;
    private double m_previousClimbKI;
    private double m_previousClimbKD;

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

        m_climbPIDTuner = new PIDTuner("climb/{tuning}climbMotor", true);

        m_climbPIDTuner.setP(CLIMB_kP_1);
        m_climbPIDTuner.setI(CLIMB_kI_1);
        m_climbPIDTuner.setD(CLIMB_kD_1);
        m_climbPIDTuner.setSlot(0);
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

        if (m_climbPIDTuner.isSlotDifferent(m_previousClimbSlot)) {
            if (m_climbPIDTuner.getSlot() == 1) {
                m_climbPIDTuner.setP(CLIMB_kP_2);
                m_climbPIDTuner.setI(CLIMB_kI_2);
                m_climbPIDTuner.setD(CLIMB_kD_2);
            } else {
                m_climbPIDTuner.setP(CLIMB_kP_1);
                m_climbPIDTuner.setI(CLIMB_kI_1);
                m_climbPIDTuner.setD(CLIMB_kD_1);
            }
            setClimbPID(m_climbPIDTuner.getP(), m_climbPIDTuner.getI(), m_climbPIDTuner.getD(),
                    m_climbPIDTuner.getSlot());
        }
        if (m_climbPIDTuner.isDifferentValues(m_previousClimbKP, m_previousClimbKI, m_previousClimbKD)) {
            m_previousClimbKP = m_climbPIDTuner.getP();
            m_previousClimbKI = m_climbPIDTuner.getI();
            m_previousClimbKD = m_climbPIDTuner.getD();
            m_previousClimbSlot = m_climbPIDTuner.getSlot();
            setClimbPID(m_climbPIDTuner.getP(), m_climbPIDTuner.getI(), m_climbPIDTuner.getD(),
                    m_climbPIDTuner.getSlot());
        }
    }
}
