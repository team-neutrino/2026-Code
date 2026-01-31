package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RepeatCommand;

import static frc.robot.util.Constants.IntakeConstants.*;
import static frc.robot.util.Subsystems2026.intake;

public class IntakeFactory {
    public static Command runIntake() {
        return intake.runIntake(INTAKE_VOLTAGE);
    }

    public static Command runOuttake() {
        return intake.runIntake(OUTTAKE_VOLTAGE);
    }

    public static Command deployIntake() {
        return intake.deployIntake(DEPLOYED_POSITION);
    }

    public static Command deployAndRunIntake() {
        return intake.deployAndRunIntake(INTAKE_VOLTAGE, DEPLOYED_POSITION);
    }

    public static Command deployAndRunOuttake() {
        return intake.deployAndRunIntake(OUTTAKE_VOLTAGE, DEPLOYED_POSITION);
    }

    public static Command shakeBallsInHopper() {
        return intake.deployIntake(INTERMEDIATE_POSITION_1).until(() -> intake.isAtTarget())
                .andThen(intake.deployIntake(INTERMEDIATE_POSITION_2));
    }
}
