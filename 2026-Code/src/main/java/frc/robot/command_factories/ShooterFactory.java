package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.util.Subsystem.shooter;

import static frc.robot.util.Constants.ShooterConstants.*;

public class ShooterFactory {
    public static Command runShooter() {
        return shooter.runShooter(SHOOTING_VOLTAGE);
    }
}
