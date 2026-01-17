package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.util.Subsystem.climb;
import static frc.robot.util.Constants.ClimbConstants.*;

public class ClimbFactory {
    public static Command climbL1() {
        return climb.moveClimbCommand(L1_POSITION);
    }

    public static Command climbDown() {
        return climb.moveClimbCommand(GROUND_POSITION);
    }
}
