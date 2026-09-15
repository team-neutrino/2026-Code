package frc.robot.autos;

import choreo.auto.AutoFactory;
import static frc.robot.util.Subsystems.*;

public class AutoBase {
    public static AutoFactory autoFactory = new AutoFactory(swerve::getCurrentPose, swerve::resetPose,
            swerve::followChoreoTrajectory,
            true, swerve);

}
