package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;

import static frc.robot.util.Subsystems.climb;

import static frc.robot.util.Constants.ClimbConstants.*;

public class ClimbFactory {
    public static Command raiseClimbArm() {
        return climb.moveClimbCommand(RAISE_POSITION, RAISE_UP_SLOT);
    }

    public static Command lowerClimbArm() {
        return climb.moveClimbCommand(LOWER_POSITION, RAISE_UP_SLOT);
    }

    public static Command climbOnToBar() {
        return climb.moveClimbCommand(CLIMB_POSITION, RAISE_UP_SLOT);
    }

    public static Command releaseClimbFromBar() {
        return climb.moveClimbCommand(RELEASE_POSITION, LOWER_DOWN_SLOT);
    }

    public static Command climbSequentialCommand() {
        return raiseClimbArm().until(() -> climb.atCANRangeClimbPosition())
                .andThen(climbOnToBar());
    }

    public static Command climbReleaseCommand() {
        return releaseClimbFromBar().until(() -> climb.atTargetPosition())
                .andThen(lowerClimbArm().until(() -> climb.atTargetPosition()));
    }
}
