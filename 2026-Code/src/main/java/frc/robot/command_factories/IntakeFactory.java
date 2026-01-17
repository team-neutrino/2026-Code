package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.util.Subsystem.intake;

public class IntakeFactory {
    public static Command runIntake(){
        return intake.runIntake();
    }

    public static Command runOuttake(){
        return intake.runOuttake();
    }

    public static Command deployIntake(){
        return intake.deployIntake();
    }

    public static Command retractIntake(){
        return intake.retractIntake();
    }
}
