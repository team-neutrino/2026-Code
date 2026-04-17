package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;

import static frc.robot.util.Constants.IntakeConstants.*;
import static frc.robot.util.Subsystems.intake;

public class IntakeFactory {
    public static Command runIntake() {
        return intake.runIntake(INTAKE_VOLTAGE);
    }

    public static Command runOuttake() {
        return intake.runIntake(OUTTAKE_VOLTAGE);
    }

    public static Command toggleIntake() {
        return intake.toggleIntake();
    }

    public static Command shakeHopper() {
        return intake.moveIntakeIntermediate(INTERMEDIATE_POSITION_1, INTAKE_VOLTAGE);
    }

    public static Command deployAndRunIntake() {
        return intake.deployAndRunIntake(INTAKE_VOLTAGE);
    }
}
