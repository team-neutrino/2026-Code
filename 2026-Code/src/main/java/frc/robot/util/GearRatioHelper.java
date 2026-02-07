package frc.robot.util;

import static frc.robot.util.Constants.ShooterConstants.HOOD_GEAR_RATIO;

public class GearRatioHelper {
    /**
     * This method takes an angle in degrees and converts it to a value in
     * rotations, respecting the gear ratio of the hood.
     * 
     * @param degrees An angle in degrees to set the hood to.
     * @return The amount of rotations to set the hood to the specified amount of
     *         degrees, accounted for the gear ratio of the hood.
     */
    public static double convertToHood(double degrees) {
        return HOOD_GEAR_RATIO * (degrees / 360);
    }
}
