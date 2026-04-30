package frc.robot.util;

import frc.robot.subsystems.*;
import frc.robot.subsystems.NetworkTables.*;

public class Subsystems {
    public static final HubActiveStatus hubState = new HubActiveStatus();
    public static final DriverDashboard driverDashboard = new DriverDashboard();

    public static final Climb climb = null;
    public static final Intake intake = new Intake();
    public static final Index index = new Index();
    public static final Vision Vision = new Vision();
    public static final Swerve swerve = new Swerve();
    // public static final Shooter shooter = new Shooter();
    // public static final Turret turret = new TurretNT();

    public static final ShooterArbiter shooterArbiter = new ShooterArbiter();
}
