package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.Constants.ShooterConstants.fakeEnum;

import static frc.robot.util.Subsystems2026.shooter;

import static frc.robot.util.Constants.ShooterConstants.*;

public class ShooterFactory {
    /**
     * A command to prepare to shoot based on a fixed point.
     * 
     * @param target The fixed point.
     * @return A command to prepare to shoot based on a fixed point.
     */
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

    /**
     * A command to prepare to shoot based on the distance from the hub and the
     * shooter interpolation tables.
     * 
     * @param distance Distance from the Hub.
     * @return A command to prepare to shoot based on distance.
     */
    public static Command shootFromInterpolationTable(double distance) {
        double speed; // please speed I need this

        if (distance > ZONE_1) {
            speed = 1500;
        } else {
            speed = 3000;
        }

        return shooter.shootingAngle(INTERPOLATION_HOOD.get(distance))
                .alongWith(shooter.shootingSpeed(speed));
    }
}
