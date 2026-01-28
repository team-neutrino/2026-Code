package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

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
        return new SequentialCommandGroup(
                intake.deployIntake(DEPLOYED_POSITION),
                intake.runIntake(INTAKE_VOLTAGE));
    }

    public static Command deployAndRunOuttake() {
        return new SequentialCommandGroup(
                intake.deployIntake(DEPLOYED_POSITION),
                intake.runIntake(OUTTAKE_VOLTAGE));
    }

    public static Command shakeBallsInHopper() {
        return new SequentialCommandGroup(
                intake.deployIntake(INTERMEDIATE_POSITION_1),
                intake.deployIntake(INTERMEDIATE_POSITION_2));
    }
}
