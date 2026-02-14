package frc.robot.subsystems.NetworkTables;

import frc.robot.util.PIDTuner;
import static frc.robot.util.Constants.ShooterConstants.*;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.Shooter;

public class ShooterNT extends Shooter {
    private PIDTuner m_shooterPIDTuner;
    private PIDTuner m_hoodPIDTuner;

    private double m_previousShootingKP;
    private double m_previousShootingKI;
    private double m_previousShootingKD;

    private double m_previousHoodKP;
    private double m_previousHoodKI;
    private double m_previousHoodKD;

    private NetworkTableInstance m_globalNT = NetworkTableInstance.getDefault();

    private DoubleTopic m_distanceTopic;
    private DoublePublisher m_distancePublisher;
    private DoubleSubscriber m_distanceSubscriber;

    private DoubleTopic m_shooterSpeedTopic;
    private DoublePublisher m_shooterSpeedPublisher;
    private DoubleSubscriber m_shooterSpeedSubscriber;

    private DoubleTopic m_shooterTargetTopic;
    private DoublePublisher m_shooterTargetPublisher;
    private DoubleSubscriber m_shooterTargetSubscriber;

    private DoubleTopic m_hoodTargetTopic;
    private DoublePublisher m_hoodTargetPublisher;
    private DoubleSubscriber m_hoodTargetSubscriber;

    private DoubleTopic m_hoodPositionTopic;
    private DoublePublisher m_hoodPositionPublisher;
    private DoubleSubscriber m_hoodPositionSubscriber;

    private DoubleTopic m_hoodCurrentTopic;
    private DoublePublisher m_hoodCurrentPublisher;
    private DoubleSubscriber m_hoodCurrentSubscriber;

    /**
     * Creates a new class to organize network tables related to the Shooter
     * subsystem.
     * 
     * @return An object to manage Shooter network tables.
     */
    public ShooterNT() {
        m_shooterPIDTuner = new PIDTuner("shooter/{tuning}shooterMotor", false);
        m_hoodPIDTuner = new PIDTuner("shooter/{tuning}hoodMotor", false);

        m_shooterPIDTuner.setP(SHOOTING_KP);
        m_shooterPIDTuner.setI(SHOOTING_KI);
        m_shooterPIDTuner.setD(SHOOTING_KD);

        m_hoodPIDTuner.setP(HOOD_KP);
        m_hoodPIDTuner.setI(HOOD_KI);
        m_hoodPIDTuner.setD(HOOD_KD);

        m_distanceTopic = m_globalNT.getDoubleTopic("shooter/{tuning}distance");
        m_distancePublisher = m_distanceTopic.publish();
        m_distanceSubscriber = m_distanceTopic.subscribe(5);

        m_distancePublisher.set(5.0);

        m_shooterSpeedTopic = m_globalNT.getDoubleTopic("shooter/shooterSpeed");
        m_shooterSpeedPublisher = m_shooterSpeedTopic.publish();
        m_shooterSpeedSubscriber = m_shooterSpeedTopic.subscribe(0.0);

        m_shooterTargetTopic = m_globalNT.getDoubleTopic("shooter/shooterTarget");
        m_shooterTargetPublisher = m_shooterTargetTopic.publish();
        m_shooterTargetSubscriber = m_shooterTargetTopic.subscribe(0.0);

        m_hoodTargetTopic = m_globalNT.getDoubleTopic("shooter/hoodTarget");
        m_hoodTargetPublisher = m_hoodTargetTopic.publish();
        m_hoodTargetSubscriber = m_hoodTargetTopic.subscribe(0.0);

        m_hoodPositionTopic = m_globalNT.getDoubleTopic("shooter/hoodPosition");
        m_hoodPositionPublisher = m_hoodPositionTopic.publish();
        m_hoodPositionSubscriber = m_hoodPositionTopic.subscribe(0.0);

        m_hoodCurrentTopic = m_globalNT.getDoubleTopic("shooter/hoodCurrent");
        m_hoodCurrentPublisher = m_hoodCurrentTopic.publish();
        m_hoodCurrentSubscriber = m_hoodCurrentTopic.subscribe(0.0);

        // m_previousShootingKP = SHOOTING_KP;
        // m_previousShootingKI = SHOOTING_KI;
        // m_previousShootingKD = SHOOTING_KD;

        // m_previousHoodKP = HOOD_KP;
        // m_previousHoodKI = HOOD_KI;
        // m_previousHoodKD = HOOD_KD;
        // setShooterPID(m_shooterPIDTuner.getP(), m_shooterPIDTuner.getI(),
        // m_shooterPIDTuner.getD());
    }

    @Override
    public void periodic() {
        super.periodic();
        if (m_shooterPIDTuner.isDifferentValues(m_previousShootingKP, m_previousShootingKI, m_previousShootingKD)) {
            m_previousShootingKP = m_shooterPIDTuner.getP();
            m_previousShootingKI = m_shooterPIDTuner.getI();
            m_previousShootingKD = m_shooterPIDTuner.getD();
            setShooterPID(m_shooterPIDTuner.getP(), m_shooterPIDTuner.getI(), m_shooterPIDTuner.getD());
        }

        if (m_hoodPIDTuner.isDifferentValues(m_previousHoodKP, m_previousHoodKI, m_previousHoodKD)) {
            m_previousHoodKP = m_hoodPIDTuner.getP();
            m_previousHoodKI = m_hoodPIDTuner.getI();
            m_previousHoodKD = m_hoodPIDTuner.getD();
            setHoodPID(m_hoodPIDTuner.getP(), m_hoodPIDTuner.getI(), m_hoodPIDTuner.getD());
        }

        setTuningDistance(m_distanceSubscriber.get());
        m_shooterSpeedPublisher.set(getShooterRPM());
        m_shooterTargetPublisher.set(getTargetRPM());
        m_hoodTargetPublisher.set(getTargetPosition() / 360);
        m_hoodPositionPublisher.set(getHoodAngle());
        m_hoodCurrentPublisher.set(getHoodCurrent());
    }

}
