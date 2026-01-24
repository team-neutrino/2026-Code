package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.util.Subsystems2026.shooter;

import static frc.robot.util.Constants.ShooterConstants.*;

public class ShooterFactory {
    public static Command shootingAngleFromFixedPosition(fakeEnum target) {
        switch (target) {
            case RADIAL_CLOSE:
                return shooter.shootingAngle(RADIAL_CLOSE_ANGLE);
            case RADIAL_FAR:
                return shooter.shootingAngle(RADIAL_FAR_ANGLE);
            case WALL:
                return shooter.shootingAngle(WALL_ANGLE);
            case DEPOT:
                return shooter.shootingAngle(DEPOT_ANGLE);
            case OUTPOST:
                return shooter.shootingAngle(OUTPOST_ANGLE);
            default:
                return shooter.shootingAngle(DEFAULT);
        }
    }

}
