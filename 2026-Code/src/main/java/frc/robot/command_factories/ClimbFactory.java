package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import static frc.robot.util.Subsystem.climb;
import static frc.robot.util.Constants.ClimbConstants.*;

public class ClimbFactory {
    public static Command raiseClimb() {
        return climb.moveClimbCommand(L1_POSITION);
    }

    public static Command autoClimb() {
        return climb.moveClimbCommand(DOWN_POSITION);
    }

    public static Command endGameClimb() {
        return new SequentialCommandGroup(climb.moveServoCommand(SERVO_ENDGAME_POSITION), climb.moveClimbCommand(DOWN_POSITION));
    }
}
