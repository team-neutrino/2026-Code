package frc.robot.subsystems.NetworkTables;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import frc.robot.subsystems.Index;

public class IndexNT extends Index {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();

    DoubleTopic spindexerVoltage = nt.getDoubleTopic("/index/spindexer_voltage");
    DoubleTopic spindexerTargetVoltage = nt.getDoubleTopic("/index/spindexer_target_voltage");
    DoubleTopic kickerVoltage = nt.getDoubleTopic("/index/kicker_voltage");
    DoubleTopic kickerTargetVoltage = nt.getDoubleTopic("/index/kicker_target_voltage");

    final DoublePublisher spindexerVoltagePub;
    final DoublePublisher spindexerTargetVoltagePub;

    final DoublePublisher kickerVoltagePub;
    final DoublePublisher kickerTargetVoltagePub;

    public IndexNT() {
        spindexerVoltagePub = spindexerVoltage.publish();
        spindexerVoltagePub.setDefault(0.0);

        spindexerTargetVoltagePub = spindexerTargetVoltage.publish();
        spindexerTargetVoltagePub.setDefault(0.0);

        kickerVoltagePub = kickerVoltage.publish();
        kickerVoltagePub.setDefault(0.0);

        kickerTargetVoltagePub = kickerTargetVoltage.publish();
        kickerTargetVoltagePub.setDefault(0.0);
    }

    @Override
    public void periodic() {
        super.periodic();
        final long now = NetworkTablesJNI.now();

        spindexerVoltagePub.set(getSpindexerVoltage(), now);
        spindexerTargetVoltagePub.set(getSpindexerTargetVoltage(), now);

        kickerVoltagePub.set(getKickerVoltage(), now);
        kickerTargetVoltagePub.set(getKickerTargetVoltage(), now);
    }
}
