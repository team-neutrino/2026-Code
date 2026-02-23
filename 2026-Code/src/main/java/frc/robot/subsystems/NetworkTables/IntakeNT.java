package frc.robot.subsystems.NetworkTables;

import static frc.robot.util.Constants.IntakeConstants.*;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import frc.robot.subsystems.Intake;
import frc.robot.util.PIDTuner;

public class IntakeNT extends Intake {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();

    DoubleTopic currentDeployPosition = nt.getDoubleTopic("/intake/current_intake_position");
    DoubleTopic targetDeployPosition = nt.getDoubleTopic("/intake/target_intake_position");

    final DoublePublisher currentDeployPositionPub;
    final DoublePublisher targetDeployPositionPub;

    private PIDTuner m_deployPIDTuner;

    private double m_previousDeployKP;
    private double m_previousDeployKI;
    private double m_previousDeployKD;

    public IntakeNT() {
        currentDeployPositionPub = currentDeployPosition.publish();
        currentDeployPositionPub.setDefault(0.0);

        targetDeployPositionPub = targetDeployPosition.publish();
        targetDeployPositionPub.setDefault(0.0);

        m_deployPIDTuner = new PIDTuner("intake/{tuning}deployMotor", false);

        m_deployPIDTuner.setP(INTAKE_kP);
        m_deployPIDTuner.setI(INTAKE_kI);
        m_deployPIDTuner.setD(INTAKE_kD);
    }

    @Override
    public void periodic() {
        super.periodic();
        final long now = NetworkTablesJNI.now();

        currentDeployPositionPub.set(getMotorAngle(), now);
        targetDeployPositionPub.set(getTargetAngle(), now);

        if (m_deployPIDTuner.isDifferentValues(m_previousDeployKP, m_previousDeployKI, m_previousDeployKD)) {
            m_previousDeployKP = m_deployPIDTuner.getP();
            m_previousDeployKI = m_deployPIDTuner.getI();
            m_previousDeployKD = m_deployPIDTuner.getD();
            setIntakePID(m_deployPIDTuner.getP(), m_deployPIDTuner.getI(), m_deployPIDTuner.getD());
        }
    }
}
