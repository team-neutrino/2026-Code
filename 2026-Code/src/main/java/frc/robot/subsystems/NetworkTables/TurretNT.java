package frc.robot.subsystems.NetworkTables;

import static frc.robot.util.Constants.GlobalConstants.RED_ALLIANCE;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructTopic;
import frc.robot.subsystems.Turret;
import frc.robot.util.PIDTuner;
import frc.robot.util.Subsystems;
import frc.robot.util.Constants.TurretConstants;

public class TurretNT extends Turret {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    DoubleTopic encoderPosition = nt.getDoubleTopic("/turret/encoder_position");
    DoubleTopic targetPosition = nt.getDoubleTopic("/turret/target_position");
    DoubleTopic encoderVelocity = nt.getDoubleTopic("/turret/encoder_velocity");
    BooleanTopic scoreReady = nt.getBooleanTopic("/turret/at_target");
    StructTopic<Pose2d> turretPose = nt.getStructTopic("/turret/pose", Pose2d.struct);

    final DoublePublisher encoderPositionPub;
    final DoublePublisher targetPositionPub;
    final DoublePublisher encoderVelocityPub;
    final BooleanPublisher scoreReadyPub;
    final StructPublisher<Pose2d> turretPosePub;
    private PIDTuner m_PIDTuner;
    private double m_previousP = TurretConstants.TURRET_P;
    private double m_previousI = TurretConstants.TURRET_I;
    private double m_previousD = TurretConstants.TURRET_D;
    private Pose2d blank = new Pose2d();

    public TurretNT() {
        encoderPositionPub = encoderPosition.publish();
        encoderPositionPub.setDefault(0.0);

        targetPositionPub = targetPosition.publish();
        targetPositionPub.setDefault(0.0);

        encoderVelocityPub = encoderVelocity.publish();
        encoderVelocityPub.setDefault(0.0);

        scoreReadyPub = scoreReady.publish();
        scoreReadyPub.setDefault(false);

        turretPosePub = turretPose.publish();
        turretPosePub.set(blank);

        m_PIDTuner = new PIDTuner("turret/{tuning}PIDF", false);

        m_PIDTuner.setP(m_previousP);
        m_PIDTuner.setI(m_previousI);
        m_PIDTuner.setD(m_previousD);
    }

    @Override
    public void periodic() {
        super.periodic();
        final long now = NetworkTablesJNI.now();
        encoderPositionPub.set(getCurrentAngle(), now);
        turretPosePub.set(
                new Pose2d(Subsystems.swerve.getTurretGlobal(), new Rotation2d(Subsystems.swerve.getYawRadians())),
                now);
        if (RED_ALLIANCE.isPresent()) {
            targetPositionPub.set(getAdjustedTargetAngle(), now);
        }
        scoreReadyPub.set(isAtTarget(), now);

        if (m_PIDTuner.isDifferentValues(m_previousP, m_previousI, m_previousD)) {
            changePID(m_PIDTuner.getP(), m_PIDTuner.getI(), m_PIDTuner.getD());
            m_previousP = m_PIDTuner.getP();
            m_previousI = m_PIDTuner.getI();
            m_previousD = m_PIDTuner.getD();
        }

    }
}
