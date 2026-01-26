package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import static frc.robot.util.Constants.IntakeConstants.DEPLOY_VOLTAGE;
import static frc.robot.util.Constants.IntakeConstants.INTAKE_VOLTAGE;
import static frc.robot.util.Constants.IntakeConstants.OUTTAKE_VOLTAGE;
import static frc.robot.util.Constants.IntakeConstants.RETRACT_VOLTAGE;
import static frc.robot.util.Subsystems2026.intake;

public class IntakeFactory {
    public static Command runIntake() {
        return intake.runIntake(INTAKE_VOLTAGE);
    }

    public static Command runOuttake() {
        return intake.runIntake(OUTTAKE_VOLTAGE);
    }

    public static Command deployIntake() {
        return intake.deployIntake(DEPLOY_VOLTAGE);
    }

    public static Command retractIntake() {
        return intake.deployIntake(RETRACT_VOLTAGE);
    }

    public static Command deployAndRunIntake() {
        return new SequentialCommandGroup(
                intake.deployIntake(DEPLOY_VOLTAGE),
                intake.runIntake(INTAKE_VOLTAGE));
    }

    public static Command deployAndRunOuttake() {
        return new SequentialCommandGroup(
                intake.deployIntake(DEPLOY_VOLTAGE),
                intake.runIntake(OUTTAKE_VOLTAGE));
    }
}
