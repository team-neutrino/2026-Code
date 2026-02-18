package frc.robot.subsystems.NetworkTables;

import static frc.robot.util.Constants.GlobalConstants.RED_ALLIANCE;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import frc.robot.subsystems.Turret;
import frc.robot.util.PIDTuner;
import frc.robot.util.Constants.TurretConstants;
import frc.robot.util.FFTuner;
import frc.robot.util.MotionMagicTuner;

public class TurretNT extends Turret {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    DoubleTopic encoderPosition = nt.getDoubleTopic("/turret/encoder_position");
    DoubleTopic targetPosition = nt.getDoubleTopic("/turret/target_position");
    DoubleTopic encoderVelocity = nt.getDoubleTopic("/turret/encoder_velocity");
    BooleanTopic scoreReady = nt.getBooleanTopic("/turret/at_target");
    final DoublePublisher encoderPositionPub;
    final DoublePublisher targetPositionPub;
    final DoublePublisher encoderVelocityPub;
    final BooleanPublisher scoreReadyPub;
    private PIDTuner m_PIDTuner;
    private double m_previousP = TurretConstants.TURRET_P;
    private double m_previousI = TurretConstants.TURRET_I;
    private double m_previousD = TurretConstants.TURRET_D;
    private double m_previousFF = TurretConstants.TURRET_FF;
    private FFTuner m_FFTuner;
    private MotionMagicTuner m_motionMagicTuner;
    private double m_previousMaxAcceleration = TurretConstants.TARGET_ACCELERATION;
    private double m_previousMaxVelocity = TurretConstants.TARGET_CRUISE_VELOCITY;
    private double m_previousMaxJerk = TurretConstants.TARGET_JERK;

    public TurretNT() {
        encoderPositionPub = encoderPosition.publish();
        encoderPositionPub.setDefault(0.0);

        targetPositionPub = targetPosition.publish();
        targetPositionPub.setDefault(0.0);

        encoderVelocityPub = encoderVelocity.publish();
        encoderVelocityPub.setDefault(0.0);

        scoreReadyPub = scoreReady.publish();
        scoreReadyPub.setDefault(false);

        m_PIDTuner = new PIDTuner("turret/{tuning}PIDF", false);

        m_PIDTuner.setP(m_previousP);
        m_PIDTuner.setI(m_previousI);
        m_PIDTuner.setD(m_previousD);

        m_FFTuner = new FFTuner("turret/{tuning}PIDF");

        m_FFTuner.setFF(m_previousFF);

        m_motionMagicTuner = new MotionMagicTuner("turret/{tuning}MotionMagic");

        m_motionMagicTuner.setVelocity(m_previousMaxVelocity);
        m_motionMagicTuner.setAcceleration(m_previousMaxAcceleration);
        m_motionMagicTuner.setJerk(m_previousMaxJerk);
    }

    @Override
    public void periodic() {
        super.periodic();
        final long now = NetworkTablesJNI.now();
        encoderPositionPub.set(getCurrentAngle(), now);
        if (RED_ALLIANCE.isPresent()) {
            targetPositionPub.set(getTargetAngle(), now);
        }
        scoreReadyPub.set(isAtTarget(), now);

        if (m_PIDTuner.isDifferentValues(m_previousP, m_previousI, m_previousD)) {
            changePID(m_PIDTuner.getP(), m_PIDTuner.getI(), m_PIDTuner.getD());
            m_previousP = m_PIDTuner.getP();
            m_previousI = m_PIDTuner.getI();
            m_previousD = m_PIDTuner.getD();
        }

        if (m_FFTuner.getFF() != m_previousFF) {
            changeFF(m_FFTuner.getFF());
            m_previousFF = m_FFTuner.getFF();
        }

        if (m_motionMagicTuner.isDifferentValues(m_previousMaxVelocity, m_previousMaxAcceleration,
                m_previousMaxJerk)) {
            changeMotionmagic(m_motionMagicTuner.getVelocity(), m_motionMagicTuner.getAcceleration(),
                    m_motionMagicTuner.getJerk());
            m_previousMaxVelocity = m_motionMagicTuner.getVelocity();
            m_previousMaxAcceleration = m_motionMagicTuner.getAcceleration();
            m_previousMaxJerk = m_motionMagicTuner.getJerk();
        }
    }
}
