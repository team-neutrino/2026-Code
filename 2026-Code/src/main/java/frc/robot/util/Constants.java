package frc.robot.util;

import java.util.List;
import java.util.Optional;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Constants {
    public static class RioConstants {
        public static final CANBus RIO_BUS = new CANBus("rio");
    }

    public static class GlobalConstants {
        public static Optional<Boolean> RED_ALLIANCE = Optional.empty();
    }

    public static class ClimbConstants {
        public static final int CLIMB_MOTOR_ID_1 = 18;
        public static final int CLIMB_CURRENT_LIMIT = 40;

        public static final double CLIMB_kP = 0.3;
        public static final double CLIMB_kI = 0.1;
        public static final double CLIMB_kD = 0.0;
        public static final double CLIMB_kFF = 0.0;

        public static final double ALLOWED_ERROR = 0.1;

        public static final double L1_POSITION = 10;
        public static final double DOWN_POSITION = 0;

        public static final int CANANDCOLOR_ID = 28;
        public static final double CANANDCOLOR_DISTANCE = 0.1;

        public static final int CANRANGE_ID = 21;

        public static final int CLIMB_SERVO_PORT = 0;
        public static final int SERVO_ENDGAME_POSITION = 1;
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

    public static class AlphabotIntakeConstants {
        public static final int INTAKE_MOTOR_ID = 13;
        public static final int INDEX_MOTOR_ID = 14;

        public static final double CURRENT_LIMIT = 60;
        public static final double INTAKE_VOLTAGE = 6;
        public static final double INDEX_VOLTAGE = 0;
    }

    public static class KickerConstants {
        public static final int KICKER_MOTOR_ID = 18;
        public static final double KICKER_CURRENT_LIMIT = 40;
        public static final double KICKER_VOLTAGE = 8;
    }

    public static class AlphabotKickerConstants {
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
        public static final double BL_SIDE_OFFSET = 0; // Side offset (meters) right is positive
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
        public static final String AlphaLL_BR = "limelight-mlksrbr"; // 3G
        public static final String AlphaLL_BL = "limelight-mlksrbl"; // 4
        public static final String AlphaLL_SHOOTER = "limelight-ashoot"; // 4

        // Camera pose offsets
        public static final double AlphaBR_FORWARD_OFFSET = -0.1397;
        public static final double AlphaBR_SIDE_OFFSET = 0.307975;
        public static final double AlphaBR_HEIGHT_OFFSET = 0.238125;
        public static final double AlphaBR_ROLL_OFFSET = 180; // Roll (degrees)
        public static final double AlphaBR_PITCH_OFFSET = 30; // Pitch (degrees)
        public static final double AlphaBR_YAW_OFFSET = 180; // Yaw (degrees)

        public static final double AlphaBL_FORWARD_OFFSET = -0.1397; // Forward offset (meters)
        public static final double AlphaBL_SIDE_OFFSET = -0.307975; // Side offset (meters) right is positive
        public static final double AlphaBL_HEIGHT_OFFSET = .254; // Height offset (meters)
        public static final double AlphaBL_ROLL_OFFSET = 0; // Roll (degrees)
        public static final double AlphaBL_PITCH_OFFSET = 30; // Pitch (degrees)
        public static final double AlphaBL_YAW_OFFSET = 180; // Yaw (degrees)

        public static final double AlphaSHOOTER_FORWARD_OFFSET = -0.03048;
        public static final double AlphaSHOOTER_SIDE_OFFSET = -0.130175;
        public static final double AlphaSHOOTER_HEIGHT_OFFSET = 0.61468;
        public static final double AlphaSHOOTER_ROLL_OFFSET = 0;
        public static final double AlphaSHOOTER_PITCH_OFFSET = 15;
        public static final double AlphaSHOOTER_YAW_OFFSET = 0;

        public static final double AlphaMINIMUM_XY_STD_DEV_LL4 = 0.15;
        public static final double AlphaMINIMUM_THETA_STD_DEV_LL4 = 9999999;
        public static final double AlphaMINIMUM_XY_STD_DEV_LL3G = 0.15;
        public static final double AlphaMINIMUM_THETA_STD_DEV_LL3G = 9999999;
        public static final double AlphaERROR_FACTOR_LL4 = 0.05;
        public static final double AlphaERROR_FACTOR_LL4_ANGLE = 2;
        public static final double AlphaERROR_FACTOR_LL3G = 0.1;
        public static final double AlphaERROR_FACTOR_LL3G_ANGLE = 2;
    }

    public static class SwerveConstants {
        public static final double GYRO_SCALAR_Z = -3.9;
        public static final double MAX_SPEED = 5.7;
        public static final double MAX_ROTATION_SPEED = 1.5 * Math.PI;
        public static final double DRIVE_ASSIST_KP = 8;
        public static final double AUTO_ALIGN_D = 0;
    }

    public static class DriveToPointConstants {
        public static final double DRIVE_TO_POINT_P = 3;
        public static final double DRIVE_TO_POINT_I = 0;
        public static final double DRIVE_TO_POINT_D = 0;
        public static final double MAX_DRIVETOPOINT_SPEED = 5;

        public static final double SPLINE_MAX_SPEED = 3.0;
        public static final double SPLINE_MAX_ACCELERATION = 3.0;
        public static final double SPLINE_MAX_ANGULAR_VELOCITY = 2 * Math.PI;
        public static final double SPLINE_MAX_ANGULAR_ACCELERATION = 4 * Math.PI;
        public static final double SPLINE_END_VELOCITY = 1.0;
        // Could be different multiples of 45
        public static final Pose2d BLUE_PAST_BUMP_RIGHT = new Pose2d(5.7, 2.5, new Rotation2d(45));
        public static final Pose2d BLUE_PAST_BUMP_LEFT = new Pose2d(5.7, 5.57, new Rotation2d(45));
        public static final Pose2d RED_PAST_BUMP_LEFT = new Pose2d(10.84, 2.5, new Rotation2d(45));
        public static final Pose2d RED_PAST_BUMP_RIGHT = new Pose2d(10.84, 5.57, new Rotation2d(45));

        // Radius 3 meters
        public static final Pose2d BLUE_CENTER_SHOT = new Pose2d(1.6, 4.03, new Rotation2d(0));
        public static final Pose2d BLUE_RIGHT_SHOT = new Pose2d(2.47, 1.91, new Rotation2d(45));
        public static final Pose2d BLUE_MID_RIGHT_SHOT = new Pose2d(2, 2.53, new Rotation2d(30));
        public static final Pose2d BLUE_LEFT_SHOT = new Pose2d(2.47, 6.15, new Rotation2d(315));
        public static final Pose2d BLUE_MID_LEFT_SHOT = new Pose2d(2, 5.53, new Rotation2d(330));

        public static final Pose2d RED_CENTER_SHOT = new Pose2d(14.94, 4.03, new Rotation2d(0));
        public static final Pose2d RED_LEFT_SHOT = new Pose2d(14.07, 1.91, new Rotation2d(315));
        public static final Pose2d RED_MID_LEFT_SHOT = new Pose2d(14.54, 2.53, new Rotation2d(330));
        public static final Pose2d RED_RIGHT_SHOT = new Pose2d(14.07, 6.15, new Rotation2d(45));
        public static final Pose2d RED_MID_RIGHT_SHOT = new Pose2d(14.54, 5.53, new Rotation2d(30));

        // Climb positions (NEED TO BE UPDATED)
        public static final Pose2d BLUE_CLIMB_CENTER = new Pose2d(1, 4.03, new Rotation2d(0));
        public static final Pose2d BLUE_CLIMB_LEFT = new Pose2d(1, 4.03, new Rotation2d(0));
        public static final Pose2d BLUE_CLIMB_RIGHT = new Pose2d(1, 4.03, new Rotation2d(0));

        public static final Pose2d RED_CLIMB_CENTER = new Pose2d(16, 4.03, new Rotation2d(0));
        public static final Pose2d RED_CLIMB_LEFT = new Pose2d(16, 4.03, new Rotation2d(0));
        public static final Pose2d RED_CLIMB_RIGHT = new Pose2d(16, 4.03, new Rotation2d(0));

        // Pose lists
        public static final List<Pose2d> BLUE_RADIAL_SHOOTING_POSES = List.of(BLUE_CENTER_SHOT, BLUE_RIGHT_SHOT,
                BLUE_MID_RIGHT_SHOT,
                BLUE_LEFT_SHOT, BLUE_MID_LEFT_SHOT);
        public static final List<Pose2d> RED_RADIAL_SHOOTING_POSES = List.of(RED_CENTER_SHOT, RED_RIGHT_SHOT,
                RED_MID_RIGHT_SHOT,
                RED_LEFT_SHOT, RED_MID_LEFT_SHOT);

        public static final List<Pose2d> BLUE_NEUTRAL_ZONE_POSES = List.of(BLUE_PAST_BUMP_RIGHT, BLUE_PAST_BUMP_LEFT);
        public static final List<Pose2d> RED_NEUTRAL_ZONE_POSES = List.of(RED_PAST_BUMP_RIGHT, RED_PAST_BUMP_LEFT);

        public static final List<Pose2d> BLUE_SHUTTLE_POSES = List.of();
        public static final List<Pose2d> RED_SHUTTLE_POSES = List.of();

        public static final List<Pose2d> BLUE_CLIMB_POSES = List.of(BLUE_CLIMB_LEFT, BLUE_CLIMB_CENTER,
                BLUE_CLIMB_RIGHT);
        public static final List<Pose2d> RED_CLIMB_POSES = List.of(RED_CLIMB_LEFT, RED_CLIMB_CENTER, RED_CLIMB_RIGHT);

    }

    public static class FieldMeasurementConstants {
        public static final double ALLIANCE_ZONE_BLUE = 3.978;
        public static final double ALLIANCE_ZONE_RED = 12.563;

    }
}
