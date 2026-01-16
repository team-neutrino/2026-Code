package frc.robot.util;

import frc.robot.alpha_subsystems.AlphabotIndexer;
import frc.robot.alpha_subsystems.AlphabotIntake;
import frc.robot.alpha_subsystems.AlphabotShooter;

public class Subsystem {
    public static final AlphabotShooter alphaShooter = new AlphabotShooter();
    public static final AlphabotIntake alphaIntake = new AlphabotIntake();
    public static final AlphabotIndexer alphaIndexer = new AlphabotIndexer();
}
