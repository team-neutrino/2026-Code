package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;

import static frc.robot.util.Subsystems2026.climb;
import static frc.robot.util.Constants.ClimbConstants.*;

public class ClimbFactory {
    public static Command raiseClimb() {
        return climb.moveClimbCommand(RAISE_POSITION, RAISE_UP_SLOT);
    }

    public static Command lowerClimb() {
        return climb.moveClimbCommand(LOWER_POSITION, RAISE_UP_SLOT);
    }

    public static Command climbUp() {
        return climb.moveClimbCommand(CLIMB_POSITION, RAISE_UP_SLOT);
    }

    public static Command releaseClimb() {
        return climb.moveClimbCommand(RELEASE_POSITION, LOWER_DOWN_SLOT);
    }
}
