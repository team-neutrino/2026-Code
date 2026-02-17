package frc.robot.util;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;

public class FFTuner {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    String subsystem;
    DoubleTopic FF;
    DoublePublisher FF_Pub;
    DoubleSubscriber FF_Sub;

    public FFTuner(String subsystemName) {
        FF = nt.getDoubleTopic("/" + subsystemName + "/FF");

        FF_Pub = FF.publish();
        FF_Pub.setDefault(0.0);

        FF_Sub = FF.subscribe(0);
    }

    public double getFF() {
        return FF_Sub.get();
    }

    public void setFF(double FF) {
        FF_Pub.set(FF);
    }

    public void periodic() {
    }
}
