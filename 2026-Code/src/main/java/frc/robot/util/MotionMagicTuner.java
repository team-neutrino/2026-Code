package frc.robot.util;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;

public class MotionMagicTuner {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    DoubleTopic velocity;
    DoubleTopic acceleration;
    DoubleTopic jerk;

    DoublePublisher velocityPub;
    DoublePublisher accelerationPub;
    DoublePublisher jerkPub;

    DoubleSubscriber velocitySub;
    DoubleSubscriber accelerationSub;
    DoubleSubscriber jerkSub;

    public MotionMagicTuner(String subsystemName) {

        velocity = nt.getDoubleTopic("/" + subsystemName + "/velocity");
        acceleration = nt.getDoubleTopic("/" + subsystemName + "/acceleration");
        jerk = nt.getDoubleTopic("/" + subsystemName + "/jerk");

        velocityPub = velocity.publish();
        velocityPub.setDefault(0.0);

        accelerationPub = acceleration.publish();
        accelerationPub.setDefault(0.0);

        jerkPub = jerk.publish();
        jerkPub.setDefault(0.0);

        velocitySub = velocity.subscribe(0);
        accelerationSub = acceleration.subscribe(0);
        jerkSub = jerk.subscribe(0);
    }

    public double getVelocity() {
        return velocitySub.get();
    }

    public double getAcceleration() {
        return accelerationSub.get();
    }

    public double getJerk() {
        return jerkSub.get();
    }

    public void setVelocity(double maxVelocity) {
        velocityPub.set(maxVelocity);
    }

    public void setAcceleration(double maxAcceleration) {
        accelerationPub.set(maxAcceleration);
    }

    public void setJerk(double jerk) {
        jerkPub.set(jerk);
    }

    public boolean isDifferentValues(double previousVelocity, double previousAcceleration,
            double previousJerk) {
        return getVelocity() != previousVelocity || getAcceleration() != previousAcceleration ||
                getJerk() != previousJerk;
    }

    public void periodic() {
    }
}
