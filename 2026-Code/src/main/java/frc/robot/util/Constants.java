package frc.robot.util;

public class Constants {
    public static class ShooterConstants {
        public static final double SHOOTING_VOLTAGE = -12;
        public static final double CURRENT_LIMIT = 40;
    }

    public static class IndexerConstants {
        public static final double INDEXING_VOLTAGE = 8;
        public static final double CURRENT_LIMIT = 40;
    }

    public static class IntakeConstants {
        public static final double INTAKE_VOLTAGE = 8;
        public static final double CURRENT_LIMIT = 60;
    }

    public static class LimelightConstants{
        public static final String LL_FR = "limelight-frontRight";
        public static final String LL_FL = "limelight-frontLeft";
        public static final String LL_BR = "limelight-backRight";
        public static final String LL_BL = "limelight-backLeft";
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
        public static final double FL_HEIGHT_OFFSET = 0.0;
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
        public static final int Minimum_XY_Std_Dev_LL4 = 0;
        public static final int Minimum_Theta_Std_Dev_LL4 = 0;
        public static final int Minimum_XY_Std_Dev_LL3g = 0;
        public static final int Minimum_Theta_Std_Dev_LL3g = 0;
        public static final int Minimum_XY_Std_Dev_LL3 = 0;
        public static final int Minimum_Theta_Std_Dev_LL3 = 0;
        public static final int Minimum_XY_Std_Dev_LL2 = 0;
        public static final int Minimum_Theta_Std_Dev_LL2 = 0;
        public static final double ErrorFactor_LL4 = 0;
        public static final double ErrorFactor_LL4_Angle = 0;
        public static final double ErrorFactor_LL3 = 0;
        public static final double ErrorFactor_LL3_Angle = 0;
        public static final double ErrorFactor_LL3g = 0;
        public static final double ErrorFactor_LL3g_Angle = 0;
        public static final double ErrorFactor_LL2 = 0;
        public static final double ErrorFactor_LL2_Angle = 0;
        public static final double SHOOTER_FORWARD_OFFSET = 0;
        public static final double SHOOTER_SIDE_OFFSET = 0;
        public static final double SHOOTER_HEIGHT_OFFSET = 0;
        public static final double SHOOTER_ROLL_OFFSET = 0;
        public static final double SHOOTER_PITCH_OFFSET = 0;
        public static final double SHOOTER_YAW_OFFSET = 0;

    }
}
