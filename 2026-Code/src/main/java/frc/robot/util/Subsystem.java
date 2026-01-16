package frc.robot.util;

import frc.robot.subsystems.AlphabotIndexer;
import frc.robot.subsystems.AlphabotIntake;
import frc.robot.subsystems.AlphabotShooter;

public class Subsystem {
    public static final AlphabotShooter alphaShooter = new AlphabotShooter();
    public static final AlphabotIntake alphaIntake = new AlphabotIntake();
    public static final AlphabotIndexer alphaIndexer = new AlphabotIndexer();
}
