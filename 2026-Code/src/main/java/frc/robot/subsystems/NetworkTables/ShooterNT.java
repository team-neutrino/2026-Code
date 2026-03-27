package frc.robot.subsystems.NetworkTables;

import frc.robot.util.Constants.GlobalConstants;
import frc.robot.util.PIDTuner;
import static frc.robot.util.Constants.ShooterConstants.*;
import static frc.robot.util.Subsystems.hubState;
import static frc.robot.util.Subsystems.swerve;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import frc.robot.subsystems.Shooter;

public class ShooterNT extends Shooter {

    private NetworkTableInstance m_globalNT = NetworkTableInstance.getDefault();

    private PIDTuner m_shooterPIDTuner;
    private PIDTuner m_hoodPIDTuner;

    private double m_previousShootingKP;
    private double m_previousShootingKI;
    private double m_previousShootingKD;

    private double m_previousHoodKP;
    private double m_previousHoodKI;
    private double m_previousHoodKD;

    private DoubleTopic m_distanceTopic;
    private DoublePublisher m_distancePublisher;
    private DoubleSubscriber m_distanceSubscriber;

    private DoubleTopic m_shooterSpeedTopic;
    private DoublePublisher m_shooterSpeedPublisher;

    private DoubleTopic m_shooterTargetTopic;
    private DoublePublisher m_shooterTargetPublisher;

    private DoubleTopic m_hoodTargetTopic;
    private DoublePublisher m_hoodTargetPublisher;

    private DoubleTopic m_hoodPositionTopic;
    private DoublePublisher m_hoodPositionPublisher;

    private DoubleTopic m_hoodCurrentTopic;
    private DoublePublisher m_hoodCurrentPublisher;

    private DoubleTopic m_targetAngleTopic;
    private DoublePublisher m_targetAnglePublisher;
    private DoubleSubscriber m_targetAngleSubscriber;

    private DoubleTopic m_targetShooterRpmTopic;
    private DoublePublisher m_targetShooterRpmPublisher;
    private DoubleSubscriber m_targetShooterRpmSubscriber;

    private DoubleTopic m_realDistanceTopic;
    private DoublePublisher m_realDistancePublisher;

    private BooleanTopic m_speedAtTargetTopic;
    private BooleanPublisher m_speedAtTargetPublisher;

    private BooleanTopic m_hoodAtTargetTopic;
    private BooleanPublisher m_hoodAtTargetPublisher;

    private BooleanTopic m_inAllianceZoneTopic;
    private BooleanPublisher m_inAllianceZonePublisher;

    private BooleanTopic m_notDrivingTopic;
    private BooleanPublisher m_notDrivingPublisher;

    private BooleanTopic m_hubActiveTopic;
    private BooleanPublisher m_hubActivePublisher;

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

        m_shooterTargetTopic = m_globalNT.getDoubleTopic("shooter/shooterTarget");
        m_shooterTargetPublisher = m_shooterTargetTopic.publish();

        m_hoodTargetTopic = m_globalNT.getDoubleTopic("shooter/hoodTarget");
        m_hoodTargetPublisher = m_hoodTargetTopic.publish();

        m_hoodPositionTopic = m_globalNT.getDoubleTopic("shooter/hoodPosition");
        m_hoodPositionPublisher = m_hoodPositionTopic.publish();

        m_targetAngleTopic = m_globalNT.getDoubleTopic("shooter/tuningAngle");
        m_targetAnglePublisher = m_targetAngleTopic.publish();
        m_targetAngleSubscriber = m_targetAngleTopic.subscribe(-1.0);
        m_targetAnglePublisher.set(-1.0);

        m_targetShooterRpmTopic = m_globalNT.getDoubleTopic("shooter/tuningSpeed");
        m_targetShooterRpmPublisher = m_targetShooterRpmTopic.publish();
        m_targetShooterRpmSubscriber = m_targetShooterRpmTopic.subscribe(-1.0);
        m_targetShooterRpmPublisher.set(-1.0);

        m_hoodCurrentTopic = m_globalNT.getDoubleTopic("shooter/hoodCurrent");
        m_hoodCurrentPublisher = m_hoodCurrentTopic.publish();

        m_realDistanceTopic = m_globalNT.getDoubleTopic("shooter/distance");
        m_realDistancePublisher = m_realDistanceTopic.publish();

        m_speedAtTargetTopic = m_globalNT.getBooleanTopic("shooter/speedAtTarget");
        m_speedAtTargetPublisher = m_speedAtTargetTopic.publish();

        m_hoodAtTargetTopic = m_globalNT.getBooleanTopic("shooter/hoodAtTarget");
        m_hoodAtTargetPublisher = m_hoodAtTargetTopic.publish();

        m_inAllianceZoneTopic = m_globalNT.getBooleanTopic("shooter/inAllianceZone");
        m_inAllianceZonePublisher = m_inAllianceZoneTopic.publish();

        m_notDrivingTopic = m_globalNT.getBooleanTopic("shooter/notDriving");
        m_notDrivingPublisher = m_notDrivingTopic.publish();

        m_hubActiveTopic = m_globalNT.getBooleanTopic("shooter/hubActive");
        m_hubActivePublisher = m_hubActiveTopic.publish();

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

        if (swerve == null) {
            return;
        }
        final long now = NetworkTablesJNI.now();

        // if (m_shooterPIDTuner.isDifferentValues(m_previousShootingKP,
        // m_previousShootingKI, m_previousShootingKD)) {
        // m_previousShootingKP = m_shooterPIDTuner.getP();
        // m_previousShootingKI = m_shooterPIDTuner.getI();
        // m_previousShootingKD = m_shooterPIDTuner.getD();
        // setShooterPID(m_shooterPIDTuner.getP(), m_shooterPIDTuner.getI(),
        // m_shooterPIDTuner.getD());
        // }

        // if (m_hoodPIDTuner.isDifferentValues(m_previousHoodKP, m_previousHoodKI,
        // m_previousHoodKD)) {
        // m_previousHoodKP = m_hoodPIDTuner.getP();
        // m_previousHoodKI = m_hoodPIDTuner.getI();
        // m_previousHoodKD = m_hoodPIDTuner.getD();
        // setHoodPID(m_hoodPIDTuner.getP(), m_hoodPIDTuner.getI(),
        // m_hoodPIDTuner.getD());
        // }

        setTuningDistance(m_distanceSubscriber.get());
        m_shooterSpeedPublisher.set(getShooterRPM(), now);
        m_shooterTargetPublisher.set(getTargetRPM(), now);
        m_hoodTargetPublisher.set(getTargetPosition(), now);
        m_hoodPositionPublisher.set(getHoodAngle(), now);
        // m_hoodCurrentPublisher.set(getHoodCurrent(), now);
        m_realDistancePublisher.set(swerve.getFromHubToTurret(), now);
        // m_tuningAngle = m_targetAngleSubscriber.get();
        // m_tuningSpeed = m_targetShooterRpmSubscriber.get();
        m_speedAtTargetPublisher.set(atTargetRPM());
        m_hoodAtTargetPublisher.set(atTargetPosition());
        // m_inAllianceZonePublisher.set(!swerve.inNeutralOrOpposingZone());
        m_notDrivingPublisher.set(swerve.isNotMovingTooFastOrTurning());
        // if (hubState.hasValidGameData() && GlobalConstants.RED_ALLIANCE.isPresent())
        // {
        // if (GlobalConstants.RED_ALLIANCE.get()) {
        // m_hubActivePublisher.set(hubState.isRedHubActive(), now);
        // } else {
        // m_hubActivePublisher.set(hubState.isBlueHubActive(), now);
        // }
        // }
    }

}
