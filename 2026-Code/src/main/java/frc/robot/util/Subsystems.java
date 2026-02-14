package frc.robot.util;

import frc.robot.subsystems.*;
import frc.robot.subsystems.NetworkTables.*;

public class Subsystems {
    public static final HubActiveStatus hubState = new HubActiveStatus();

    public static final FuelUDPReceiver fuelUDPReceiver = new FuelUDPReceiver();
    public static final Climb climb = new ClimbNT();
    public static final Intake intake = new Intake();
    public static final Index index = new Index();
    public static final Kicker kicker = new Kicker();
    public static final Vision limelight = new Vision();
    public static final Shooter shooter = new ShooterNT();
    public static final Swerve swerve = new Swerve();
    public static final Turret turret = new TurretNT();

    public static final ShooterArbiter shooterArbiter = new ShooterArbiter();
}