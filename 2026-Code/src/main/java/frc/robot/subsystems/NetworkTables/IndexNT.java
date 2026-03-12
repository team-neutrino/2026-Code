package frc.robot.subsystems.NetworkTables;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import frc.robot.subsystems.Index;

public class IndexNT extends Index {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();

    DoubleTopic spindexerCurrentVoltage = nt.getDoubleTopic("/index/spindexer_voltage");
    DoubleTopic spindexerTargetVoltage = nt.getDoubleTopic("/index/spindexer_target_voltage");
    DoubleTopic kickerCurrentVoltage = nt.getDoubleTopic("/index/kicker_voltage");
    DoubleTopic kickerTargetVoltage = nt.getDoubleTopic("/index/kicker_target_voltage");
    DoubleTopic m_ballsPerSecond = nt.getDoubleTopic("/index/balls_per_second");

    final DoublePublisher spindexerCurrentVoltagePub;
    final DoublePublisher spindexerTargetVoltagePub;

    final DoublePublisher kickerCurrentVoltagePub;
    final DoublePublisher kickerTargetVoltagePub;

    final DoublePublisher m_ballsPerSecondPub;

    public IndexNT() {
        spindexerCurrentVoltagePub = spindexerCurrentVoltage.publish();
        spindexerCurrentVoltagePub.setDefault(0.0);

        spindexerTargetVoltagePub = spindexerTargetVoltage.publish();
        spindexerTargetVoltagePub.setDefault(0.0);

        kickerCurrentVoltagePub = kickerCurrentVoltage.publish();
        kickerCurrentVoltagePub.setDefault(0.0);

        kickerTargetVoltagePub = kickerTargetVoltage.publish();
        kickerTargetVoltagePub.setDefault(0.0);

        m_ballsPerSecondPub = m_ballsPerSecond.publish();
        m_ballsPerSecondPub.setDefault(0.0);
    }

    @Override
    public void periodic() {
        super.periodic();
        final long now = NetworkTablesJNI.now();

        spindexerCurrentVoltagePub.set(getSpindexerCurrentVoltage(), now);
        spindexerTargetVoltagePub.set(getSpindexerTargetVoltage(), now);

        kickerCurrentVoltagePub.set(getKickerCurrentVoltage(), now);
        kickerTargetVoltagePub.set(getKickerTargetVoltage(), now);

        if (m_bpsTimer.hasElapsed(1)) {
            m_ballsPerSecondPub.set(getBallsPerSecond(), now);
            resetTimerAndCount();
        }
    }
}
