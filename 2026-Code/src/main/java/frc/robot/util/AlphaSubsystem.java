package frc.robot.util;

import frc.robot.alpha_subsystems.AlphabotKicker;
import frc.robot.alpha_subsystems.AlphabotLimelight;
import frc.robot.alpha_subsystems.AlphabotIntake;
import frc.robot.alpha_subsystems.AlphabotShooter;
import frc.robot.subsystems.Swerve;

public class AlphaSubsystem {
    public static final HubActiveStatus hubState = new HubActiveStatus();

    public static final Swerve swerve = new Swerve();
    public static final AlphabotShooter alphaShooter = new AlphabotShooter();
    public static final AlphabotIntake alphaIntake = new AlphabotIntake();
    public static final AlphabotKicker alphaKicker = new AlphabotKicker();
    public static final AlphabotLimelight alphabotLimelight = new AlphabotLimelight();
}