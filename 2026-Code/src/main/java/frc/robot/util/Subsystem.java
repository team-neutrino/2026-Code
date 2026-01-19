package frc.robot.util;

import frc.robot.subsystems.*;

public class Subsystem {
    public static final HubActiveStatus hubState = new HubActiveStatus();

    public static final Climb climb = new Climb();
    public static final Intake intake = new Intake();
    public static final Index index = new Index();
    public static final Limelight limelight = new Limelight();
    public static final Swerve swerve = new Swerve();
}
