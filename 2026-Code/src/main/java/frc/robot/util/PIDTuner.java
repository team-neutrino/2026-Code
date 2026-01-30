package frc.robot.util;

import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.EnumSet;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.DoubleSubscriber;

// stealing code from last year? I would never
public class PIDTuner {
    private NetworkTableInstance m_globalNetworkTable = NetworkTableInstance.getDefault();

    private DoubleTopic m_P; //
    private DoubleTopic m_I;
    private DoubleTopic m_D;

    // private DoublePublisher m_P_Publisher;
    private DoublePublisher m_I_Publisher;
    private DoublePublisher m_D_Publisher;

    // private DoubleSubscriber m_P_Subscriber;
    private DoubleSubscriber m_I_Subscriber;
    private DoubleSubscriber m_D_Subscriber;

    private DoubleEntry m_P_Entry;

    /**
     * A generic class for tuning PID controllers. Uses network tables to modify
     * them during runtime.
     * 
     * @param subsystemName The name of the subsystem which needs its PID controller
     *                      tuned. This must be unique!
     * @return Returns a PIDTuner object which can be used to tune PID controllers.
     * 
     */
    public PIDTuner(String subsystemName) { // probably still the best way to do this
        m_globalNetworkTable.addListener(
                new String[] { "/" + subsystemName + "/P" },
                EnumSet.of(NetworkTableEvent.Kind.kTopic),
                event -> {
                    if (event.is(NetworkTableEvent.Kind.kPublish)) {
                        System.out.println("***************** We have just updated " + event.topicInfo.name
                                + " and its new value is "
                                + event.valueData.value.getDouble());
                    }
                });

        m_P = m_globalNetworkTable.getDoubleTopic("/" + subsystemName + "/P");
        m_I = m_globalNetworkTable.getDoubleTopic("/" + subsystemName + "/I");
        m_D = m_globalNetworkTable.getDoubleTopic("/" + subsystemName + "/D");

        m_I_Subscriber = m_I.subscribe(0.0);
        m_D_Subscriber = m_D.subscribe(0.0);

        // m_P_Publisher = m_P.publish();
        m_I_Publisher = m_I.publish();
        m_D_Publisher = m_D.publish();

        // m_P_Publisher.setDefault(0.1);
        m_I_Publisher.setDefault(0.0);
        m_D_Publisher.setDefault(0.0);

        // m_P_Subscriber = m_P.subscribe(0.1);

        m_P_Entry = m_P.getEntry(0.1);
        System.out.println("************** P ENTRY " + m_P_Entry.get());

    }

    /**
     * Get the P value from this PID tuner.
     * 
     * @return The set P value for this PID tuner.
     */
    public double getP() {
        System.out.println("************** P ENTRY " + m_P_Entry.get());
        return m_P_Entry.get();
    }

    /**
     * Get the I value from this PID tuner.
     * 
     * @return The set I value for this PID tuner.
     */
    public double getI() {
        return m_I_Subscriber.get();
    }

    /**
     * Get the D value from this PID tuner.
     * 
     * @return The set D value for this PID tuner.
     */
    public double getD() {
        return m_D_Subscriber.get();
    }

    /**
     * Sets the P value of this PID tuner.
     * 
     * @param newPValue The new P value to set the PID tuner to.
     */
    public void setP(double newPValue) {
        m_P_Entry.set(newPValue);
    }

    /**
     * Sets the I value of this PID tuner.
     * 
     * @param newIValue The new I value to set the PID tuner to.
     */
    public void setI(double newIValue) {
        m_I_Publisher.set(newIValue);
    }

    /**
     * Sets the D value of this PID tuner.
     * 
     * @param newDValue The new D value to set the PID tuner to.
     */
    public void setD(double newDValue) {
        m_P_Entry.set(newDValue);
    }

    /**
     * Checks whether or not the previous values of the PID are equal to the new
     * ones in the tuner.
     * 
     * @param previousP The current P value that should be compared to the new one.
     * @param previousI The current I value that should be compared to the new one.
     * @param previousD The current D value that should be compared to the new one.
     * @return A boolean describing the result of the comparison.
     */
    public boolean isDifferentValues(double previousP, double previousI, double previousD) {
        return getP() != previousP || getI() != previousI ||
                getD() != previousD;
    }

    /**
     * Likely unnecessary, but I'm not taking my chances
     */
    public void periodic() {

    }
}
