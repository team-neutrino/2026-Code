package frc.robot.subsystems.NetworkTables;

import frc.robot.util.PIDTuner;
import static frc.robot.util.Constants.ShooterConstants.*;

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

    /** Creates a new class to organize network tables related to the Shooter subsystem.
     * @return An object to manage Shooter network tables.
     */
    public ShooterNT() {
        m_shooterPIDTuner = new PIDTuner("shooter/{tuning}shooterMotor");
        m_hoodPIDTuner = new PIDTuner("shooter/{tuning}hoodMotor");

        m_shooterPIDTuner.setP(SHOOTING_KP);
        m_shooterPIDTuner.setI(SHOOTING_KI);
        m_shooterPIDTuner.setD(SHOOTING_KD);

        m_hoodPIDTuner.setP(HOOD_KP);
        m_hoodPIDTuner.setI(HOOD_KI);
        m_hoodPIDTuner.setD(HOOD_KD);

        m_previousShootingKP = SHOOTING_KP;
        m_previousShootingKI = SHOOTING_KI;
        m_previousShootingKD = SHOOTING_KD;

        m_previousHoodKP = HOOD_KP;
        m_previousHoodKI = HOOD_KI;
        m_previousHoodKD = HOOD_KD;
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
    }



}
