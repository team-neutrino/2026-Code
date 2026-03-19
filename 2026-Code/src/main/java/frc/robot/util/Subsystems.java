package frc.robot.util;

import frc.robot.subsystems.*;
import frc.robot.subsystems.NetworkTables.*;

public class Subsystems {
    public static final HubActiveStatus hubState = new HubActiveStatus();
    public static final DriverDashboard driverDashboard = new DriverDashboard();

    public static final Climb climb = null;
    public static final Intake intake = new IntakeNT();
    public static final Index index = new IndexNT();
    public static final Vision Vision = new Vision();
    public static final ShootWhileMovingEstimatedHubPose swerve = new ShootWhileMovingEstimatedHubPose();
    public static final Shooter shooter = new ShooterNT();
    public static final Turret turret = new TurretNT();

    public static final ShooterArbiter shooterArbiter = new ShooterArbiter();
}
