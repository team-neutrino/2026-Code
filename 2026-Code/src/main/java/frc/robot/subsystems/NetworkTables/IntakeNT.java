package frc.robot.subsystems.NetworkTables;

import static frc.robot.util.Constants.IntakeConstants.INTAKE_kD;
import static frc.robot.util.Constants.IntakeConstants.INTAKE_kI;
import static frc.robot.util.Constants.IntakeConstants.INTAKE_kP;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanTopic;
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
    BooleanTopic atTargetDeployPosition = nt.getBooleanTopic("/intake/at_target_intake_position");

    final DoublePublisher currentDeployPositionPub;
    final DoublePublisher targetDeployPositionPub;
    final BooleanPublisher atTargetDeployPositionPub;

    private PIDTuner m_deployPIDTuner;

    private double m_previousDeployKP;
    private double m_previousDeployKI;
    private double m_previousDeployKD;

    public IntakeNT() {
        currentDeployPositionPub = currentDeployPosition.publish();
        currentDeployPositionPub.setDefault(0.0);

        targetDeployPositionPub = targetDeployPosition.publish();
        targetDeployPositionPub.setDefault(0.0);

        atTargetDeployPositionPub = atTargetDeployPosition.publish();
        atTargetDeployPositionPub.setDefault(false);

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
        atTargetDeployPositionPub.set(isAtTarget(), now);

        if (m_deployPIDTuner.isDifferentValues(m_previousDeployKP, m_previousDeployKI, m_previousDeployKD)) {
            m_previousDeployKP = m_deployPIDTuner.getP();
            m_previousDeployKI = m_deployPIDTuner.getI();
            m_previousDeployKD = m_deployPIDTuner.getD();
            setIntakePID(m_deployPIDTuner.getP(), m_deployPIDTuner.getI(), m_deployPIDTuner.getD());
        }
    }
}
