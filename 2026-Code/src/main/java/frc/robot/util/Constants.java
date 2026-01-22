package frc.robot.util;

import java.util.Optional;

public class Constants {
    public static class GlobalConstants{
        public static Optional<Boolean> RED_ALLIANCE = Optional.empty();
    }

    public static class ClimbConstants{
        public static final int CLIMB_MOTOR_ID_1 = 18;
        public static final int CLIMB_CURRENT_LIMIT = 40;

        public static final double CLIMB_kP = 0.0;
        public static final double CLIMB_kI = 0.0;
        public static final double CLIMB_kD = 0.0;

        public static final double ALLOWED_ERROR = 0.1;

        public static final double L1_POSITION = 10;
        public static final double GROUND_POSITION = 0;

        public static final int CANANDCOLOR_ID = 20;
        public static final double CANANDCOLOR_DISTANCE = 0.05;

        public static final int CANRANGE_ID = 21;
    }

    public static class ShooterConstants {
        public static final double SHOOTING_VOLTAGE = -12;
        public static final double CURRENT_LIMIT = 40;
    }

    public static class IndexerConstants {
        public static final int SPINDEXER_MOTOR_ID = 14;
        public static final double INDEXING_VOLTAGE = 8;
        public static final double CURRENT_LIMIT = 40;
    }

    public static class IntakeConstants {
        public static final int ROLLER_MOTOR_ID = 12;
        public static final int DEPLOY_MOTOR_ID = 13;
        public static final double CURRENT_LIMIT = 60;
        public static final double INTAKE_VOLTAGE = 0;
        public static final double OUTTAKE_VOLTAGE = 0;
        public static final double DEPLOY_VOLTAGE = 0;
        public static final double RETRACT_VOLTAGE = 0;
    }

    public static class KickerConstants {
        public static final int KICKER_MOTOR_ID = 18;
        public static final double KICKER_CURRENT_LIMIT = 40;
        public static final double KICKER_VOLTAGE = 8;
    }

    public static class LimelightConstants {
        public static final String LL_FR = "limelight-fr";
        public static final String LL_FL = "limelight-fl";
        public static final String LL_BR = "limelight-br";
        public static final String LL_BL = "limelight-bl";
        public static final String LL_SHOOTER = "limelight-shooter";

        // Camera pose offsets
        public static final double FR_FORWARD_OFFSET = 0.0;
        public static final double FR_SIDE_OFFSET = 0.0;
        public static final double FR_HEIGHT_OFFSET = 0.0;
        public static final double FR_ROLL_OFFSET = 0.0;
        public static final double FR_PITCH_OFFSET = 0.0;
        public static final double FR_YAW_OFFSET = 0.0;

        public static final double FL_FORWARD_OFFSET = 0.0;
        public static final double FL_SIDE_OFFSET = 0.0;
        public static final double FL_HEIGHT_OFFSET = 0.80;
        public static final double FL_ROLL_OFFSET = 0.0;
        public static final double FL_PITCH_OFFSET = 0.0;
        public static final double FL_YAW_OFFSET = 0.0;

        public static final double BR_FORWARD_OFFSET = 0.0;
        public static final double BR_SIDE_OFFSET = 0.0;
        public static final double BR_HEIGHT_OFFSET = 0.0;
        public static final double BR_ROLL_OFFSET = 0; // Roll (degrees)
        public static final double BR_PITCH_OFFSET = 0; // Pitch (degrees)
        public static final double BR_YAW_OFFSET = 0; // Yaw (degrees)

        public static final double BL_FORWARD_OFFSET = 0; // Forward offset (meters)
        public static final double BL_SIDE_OFFSET = 0; // Side offset (meters) left is positive
        public static final double BL_HEIGHT_OFFSET = 0; // Height offset (meters)
        public static final double BL_ROLL_OFFSET = 0; // Roll (degrees)
        public static final double BL_PITCH_OFFSET = 0; // Pitch (degrees)
        public static final double BL_YAW_OFFSET = 0; // Yaw (degrees)
        public static final double MINIMUM_XY_STD_DEV_LL4 = 0;
        public static final double MINIMUM_THETA_STD_DEV_LL4 = 0;
        public static final double MINIMUM_XY_STD_DEV_LL3G = 0;
        public static final double MINIMUM_THETA_STD_DEV_LL3G = 0;
        public static final double MINIMUM_XY_STD_DEV_LL3 = 0;
        public static final double MINIMUM_THETA_STD_DEV_LL3 = 0;
        public static final double MINIMUM_XY_STD_DEV_LL2 = 0;
        public static final double MINIMUM_THETA_STD_DEV_LL2 = 0;
        public static final double ERROR_FACTOR_LL4 = 0;
        public static final double ERROR_FACTOR_LL4_ANGLE = 0;
        public static final double ERROR_FACTOR_LL3 = 0;
        public static final double ERROR_FACTOR_LL3_ANGLE = 0;
        public static final double ERROR_FACTOR_LL3G = 0;
        public static final double ERROR_FACTOR_LL3G_ANGLE = 0;
        public static final double ERROR_FACTOR_LL2 = 0;
        public static final double ERROR_FACTOR_LL2_ANGLE = 0;
        public static final double SHOOTER_FORWARD_OFFSET = 0;
        public static final double SHOOTER_SIDE_OFFSET = 0;
        public static final double SHOOTER_HEIGHT_OFFSET = 0;
        public static final double SHOOTER_ROLL_OFFSET = 0;
        public static final double SHOOTER_PITCH_OFFSET = 0;
        public static final double SHOOTER_YAW_OFFSET = 0;
    }

    public static class AlphabotLimelightConstants {
        public static final String AlphaLL_BR = "limelight-abr"; // 3G
        public static final String AlphaLL_BL = "limelight-abl"; // 4
        public static final String AlphaLL_SHOOTER = "limelight-ashoot"; // 4

        // Camera pose offsets
        public static final double AlphaBR_FORWARD_OFFSET = -0.1397;
        public static final double AlphaBR_SIDE_OFFSET = -0.307975;
        public static final double AlphaBR_HEIGHT_OFFSET = 0.238125;
        public static final double AlphaBR_ROLL_OFFSET = 180; // Roll (degrees)
        public static final double AlphaBR_PITCH_OFFSET = 30; // Pitch (degrees)
        public static final double AlphaBR_YAW_OFFSET = 0; // Yaw (degrees)

        public static final double AlphaBL_FORWARD_OFFSET = -0.1397; // Forward offset (meters)
        public static final double AlphaBL_SIDE_OFFSET = 0.307975; // Side offset (meters) left is positive
        public static final double AlphaBL_HEIGHT_OFFSET = .254; // Height offset (meters)
        public static final double AlphaBL_ROLL_OFFSET = 0; // Roll (degrees)
        public static final double AlphaBL_PITCH_OFFSET = 30; // Pitch (degrees)
        public static final double AlphaBL_YAW_OFFSET = 0; // Yaw (degrees)

        public static final double AlphaSHOOTER_FORWARD_OFFSET = -0.03048;
        public static final double AlphaSHOOTER_SIDE_OFFSET = 0.130175;
        public static final double AlphaSHOOTER_HEIGHT_OFFSET = 0.61468;
        public static final double AlphaSHOOTER_ROLL_OFFSET = 0;
        public static final double AlphaSHOOTER_PITCH_OFFSET = 345;
        public static final double AlphaSHOOTER_YAW_OFFSET = 0;

        public static final double AlphaMINIMUM_XY_STD_DEV_LL4 = 0.4;
        public static final double AlphaMINIMUM_THETA_STD_DEV_LL4 = 9999999;
        public static final double AlphaMINIMUM_XY_STD_DEV_LL3G = 0.6;
        public static final double AlphaMINIMUM_THETA_STD_DEV_LL3G = 9999999;
        public static final double AlphaERROR_FACTOR_LL4 = 0.14;
        public static final double AlphaERROR_FACTOR_LL4_ANGLE = 2;
        public static final double AlphaERROR_FACTOR_LL3G = .21;
        public static final double AlphaERROR_FACTOR_LL3G_ANGLE = 2;
    }

    public static class SwerveConstants {
        public static final double GYRO_SCALAR_Z = -3.9;
        public static final double MAX_SPEED = 5.7;
        public static final double MAX_ROTATION_SPEED = 1.5 * Math.PI;
        public static final double DRIVE_ASSIST_KP = 8;
        public static final double AUTO_ALIGN_D = 0;
    }
}
