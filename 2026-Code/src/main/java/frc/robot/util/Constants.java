package frc.robot.util;

import static edu.wpi.first.units.Units.Meter;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;

public class Constants {
    public static class RioConstants {
        public static final CANBus RIO_BUS = new CANBus("rio");
    }

    public static class GlobalConstants {
        public static Optional<Boolean> RED_ALLIANCE = Optional.empty();
    }

    public static class ClimbConstants {
        public static final int CLIMB_MOTOR_ID = 19;
        public static final int CLIMB_CURRENT_LIMIT = 40;

        public static final double CLIMB_kP_1 = 0.3;
        public static final double CLIMB_kI_1 = 0.1;
        public static final double CLIMB_kD_1 = 0.0;
        public static final double CLIMB_kP_2 = 0.1;
        public static final double CLIMB_kI_2 = 0.1;
        public static final double CLIMB_kD_2 = 0.0;
        public static final double CLIMB_kFF = 0.0;

        public static final double ALLOWED_ERROR = 0.1;

        public static final double RAISE_POSITION = 5;
        public static final double LOWER_POSITION = 0;
        public static final double CLIMB_POSITION = 3;
        public static final double RELEASE_POSITION = 10;

        public static final int RAISE_UP_SLOT = 0;
        public static final int LOWER_DOWN_SLOT = 1;

        public static final int CANRANGE_ID = 20;

        public static final double CANRANGE_THRESHOLD = 0.6;
        public static final double CANRANGE_HYSTERSIS = 0.05;

        public static final double CLIMB_VELOCITY = 4.0;
        public static final double DISTANCE_FROM_CLIMB = 0.1;
    }

    public static class ShooterConstants {
        public static final double SHOOTER_CURRENT_LIMIT = 40;
        public static final double HOOD_CURRENT_LIMIT = 30;

        public static final double SHOOTING_KP = 0.63;
        public static final double SHOOTING_KI = 0.0;
        public static final double SHOOTING_KD = 0.0;
        public static final double SHOOTING_KV = 0.118;
        public static final double HOOD_KP = 300;
        public static final double HOOD_KI = 150.0;
        public static final double HOOD_KD = 0;

        public static final double HOOD_ALLOWED_ERROR = 2.5;
        public static final double RPM_ALLOWED_ERROR = 400;

        public static final int SHOOTER_ID = 16;
        public static final int SHOOTER_FOLLOWER_ID = 17;
        public static final int HOOD_ID = 15;

        public static final double START_POSITION = 0;
        public static final double CURRENT_SPIKE = 29.0;
        public static final double HOOD_GEAR_RATIO = 105.8239;
        public static final double SHOOT_WHILE_MOVING_VELOCITY_STARTING_THRESHOLD = 1.0;
        public static final double NOT_TURNING_THRESHOLD = 90;
        public static final double SHOOTER_RPM_NOISE = 0.5;

        public static final double SOFT_SHOT_ANGLE = 20;
        public static final double SOFT_SHOT_SPEED = 1700;
        public static final double DEFAULT_SHOOTING_SPEED = 3000;

        public static final double SHUTTLE_SHOOTING_SPEED = 4000;
        public static final double MAX_SAFE_HOOD_ANGLE = 25;

        public static final InterpolatingDoubleTreeMap HOOD_INTERPOLATION = InterpolatingDoubleTreeMap.ofEntries(
                Map.entry(0.0, 6.5),
                Map.entry(1.64, 10.0),
                Map.entry(2.33, 13.5),
                Map.entry(2.8, 15.0),
                Map.entry(3.36, 25.0),
                Map.entry(4.0, 25.0),
                Map.entry(20.0, 25.0));

        public static final InterpolatingDoubleTreeMap SPEED_INTERPOLATION = InterpolatingDoubleTreeMap.ofEntries(
                Map.entry(0.0, 3000.0),
                Map.entry(3.7, 3000.0),
                Map.entry(3.9, 3170.0),
                Map.entry(5.22, 3500.0),
                Map.entry(7.26, 4320.0));

        // may want to redo the current points in case an error was made
        public static final InterpolatingDoubleTreeMap TIME_OF_FLIGHT = InterpolatingDoubleTreeMap.ofEntries(
                Map.entry(0.6, 1.33),
                Map.entry(1.52, 1.35),
                Map.entry(2.0, 1.3),
                Map.entry(2.4, 1.28),
                Map.entry(2.8, 1.34),
                Map.entry(3.22, 1.11),
                Map.entry(3.58, 1.1),
                Map.entry(3.7, 1.07),
                Map.entry(4.14, 1.16),
                Map.entry(4.53, 1.24),
                Map.entry(5.08, 1.31),
                Map.entry(5.67, 1.43),
                Map.entry(7.26, 1.69));

        public static final InterpolatingDoubleTreeMap SHUTTLE_SPEED_INTERPOLATION = InterpolatingDoubleTreeMap
                .ofEntries(
                        Map.entry(2.0, 2900.0),
                        Map.entry(7.0, 3500.0),
                        Map.entry(11.5, 4650.0),
                        Map.entry(13.0, 8000.0),
                        Map.entry(25.0, 8000.0));

        public static enum shooterConditions {
            SHOOTER_SPEED_CORRECT,
            HOOD_ANGLE_CORRECT,
            TURRET_ANGLE_CORRECT,
            HUB_ACTIVE,
            IN_ALLIANCE_ZONE,
            SWERVE_SPEED_CORRECT,
        }
    }

    public static class TurretConstants {
        public static final int MOTOR_ID = 29;
        public static final int ENCODER_ID = 30;
        public static final double STARTUP_ANGLE = 0;
        public static final double CURRENT_LIMIT = 40;
        public static final double TURRET_P = 180;
        public static final double TURRET_I = 0;
        public static final double TURRET_D = 7;
        public static final double SENSOR_TO_MECHANISM_RATIO = 8.4;
        public static final double ROTOR_TO_SENSOR_RATIO = 12.0 / 56.0;
        public static final double TORQUE_LOAD = 0;
        public static final double ALLOWED_ERROR = 4;
        public static final double TRACKING_THRESHOLD = 67.0;
        public static final double DISCONTINUITY_POINT = 1;
        public static final double ENCODER_MAGNET_OFFSET = 0;
        public static final double STATIC_FF = 0.1;
        public static final double VELOCITY_FF = 0.0;
        public static final double ACCELERATION_FF = 0.0;
        public static final double MAX_WINDUP = 360;
        public static final double MIN_WINDUP = -180;
        public static final double TARGET_TOLERANCE = 5;
        public static final double TURRET_OFFSET_FRONT = -0.154375;
        public static final double TURRET_OFFSET_SIDE = -0.098425;
        public static final double TURRET_TRACKING_TRANSLATION_KV = 16.0;
        public static final double TURRET_TRACKING_ROTATION_KV = 22.0;
        public static final double TURRET_LATENCY = 0.0;
    }

    public static class IndexConstants {
        public static final int SPINDEXER_MOTOR_ID = 14;
        public static final int KICKER_MOTOR_ID = 18;
        public static final int CANANDCOLOR_ID = 28;

        public static final double INDEXING_VOLTAGE = -12;
        public static final double INDEX_CURRENT_LIMIT = 40;

        public static final double KICKER_VOLTAGE = 12;
        public static final double KICKER_CURRENT_LIMIT = 40;

        public static final double CANANDCOLOR_DETECT_DISTANCE = 0.05;
    }

    public static class IntakeConstants {
        public static final int ROLLER_MOTOR_ID = 12;
        public static final int DEPLOY_MOTOR_ID = 13;

        public static final double ROLLER_CURRENT_LIMIT = 60;
        public static final double DEPLOY_CURRENT_LIMIT = 35;

        public static final double INTAKE_VOLTAGE = 10;
        public static final double OUTTAKE_VOLTAGE = -10;

        public static final double STARTING_POSITION = 0;
        public static final double DEPLOYED_POSITION = -14.6;
        public static final double INTERMEDIATE_POSITION_1 = -5;
        public static final double INTERMEDIATE_POSITION_2 = -12;
        public static final double ALLOWED_TARGET_ERROR = 0.25;

        public static final double INTAKE_kP = 2.25;
        public static final double INTAKE_kI = 0.0;
        public static final double INTAKE_kD = 0.0;
    }

    public static class LimelightConstants {
        public static final String LL_FRONT = "limelight-front";
        public static final String LL_BACK = "limelight-back";
        public static final String LL_LEFT = "limelight-left";
        public static final String LL_RIGHT = "limelight-right";

        // Camera pose offsets in meters
        public static final double FRONT_FORWARD_OFFSET = 0.070646;
        public static final double FRONT_SIDE_OFFSET = -0.000635;
        public static final double FRONT_HEIGHT_OFFSET = 0.745832;
        public static final double FRONT_ROLL_OFFSET = 0.0;
        public static final double FRONT_PITCH_OFFSET = 27.5;
        public static final double FRONT_YAW_OFFSET = 0.0;

        public static final double BACK_FORWARD_OFFSET = -0.315130;
        public static final double BACK_SIDE_OFFSET = 0.098425;
        public static final double BACK_HEIGHT_OFFSET = 0.244846;
        public static final double BACK_ROLL_OFFSET = 0.0;
        public static final double BACK_PITCH_OFFSET = 30;
        public static final double BACK_YAW_OFFSET = 180;

        public static final double LEFT_FORWARD_OFFSET = -0.258801;
        public static final double LEFT_SIDE_OFFSET = -0.32166;
        public static final double LEFT_HEIGHT_OFFSET = 0.190992;
        public static final double LEFT_ROLL_OFFSET = 180;
        public static final double LEFT_PITCH_OFFSET = 30;
        public static final double LEFT_YAW_OFFSET = 87;

        public static final double RIGHT_FORWARD_OFFSET = -0.271906; // Forward offset (meters)
        public static final double RIGHT_SIDE_OFFSET = 0.265827; // Side offset (meters)
        public static final double RIGHT_HEIGHT_OFFSET = 0.211604; // Height offset (meters)
        public static final double RIGHT_ROLL_OFFSET = 0; // Roll (degrees)
        public static final double RIGHT_PITCH_OFFSET = 27; // Pitch (degrees)
        public static final double RIGHT_YAW_OFFSET = 273; // Yaw (degrees)

        public static final double MINIMUM_XY_STD_DEV_LL4 = 0.11;
        public static final double MINIMUM_THETA_STD_DEV_LL4 = 2;
        public static final double MINIMUM_XY_STD_DEV_LL3G = 0.15;
        public static final double MINIMUM_THETA_STD_DEV_LL3G = 4;
        public static final double MINIMUM_XY_STD_DEV_LL3 = 0.18;
        public static final double MINIMUM_THETA_STD_DEV_LL3 = 6;
        public static final double ERROR_FACTOR_LL4 = 0.05;
        public static final double ERROR_FACTOR_LL3 = 0.09;
        public static final double ERROR_FACTOR_LL3G = 0.07;

        public static final double IGNORE_MEASUREMENT_STD_DEV = 999999999;
        public static final double MT1_WEIGHT_YAW = .7;
        public static final double PIGEON_SEED_PERIOD = 1.0;
        public static final double PIGEON_SEED_XY_THRESHOLD = 0.2;
        public static final double EXTERNAL_WEIGHT = 0.01; // larger number (0-1) = lower trust in internal IMU
        public static final double PIGEON_SEED_DISTANCE_THRESHOLD = 3.7;
    }

    public static class SwerveConstants {
        public static final double SLEW_LIMIT = 4.0;
        public static final double GYRO_SCALAR_Z = -3.0;
        public static final double MAX_SPEED = 5.7;
        public static final double MAX_ROTATION_SPEED = 6.0; // 1.5 * Math.PI
        public static final double SLOW_MAX_ROTATION_SPEED = 4.0;
        public static final double SLOW_MAX_SPEED = 2.0;
        public static final double SLOWEST_MAX_ROTATION_SPEED = 2.0;
        public static final double SLOWEST_MAX_SPEED = 0.75;
        public static final double ROTATIONAL_P = 6.0;
        public static final double AUTO_ALIGN_D = 0.0;
        public static final double JOYSTICK_REST_ALLOWED_ERROR = 0.1;
        public static final double CONVERGENCE_ITERATIONS = 3.0;
        public static final double SHOOT_WHILE_MOVING_THRESHOLD = 2.5;
        public static final double ROBOT_WHEEL_OFFSET = 0.276225;
        public static final double BRAKE_ALLOWED_ERROR = 0.05;
        public static final double START_BRAKING_VELOCITY = 0.1;
        public static final double BEACHED_ANGLE = 5;
    }

    public static class DriveToPointConstants {
        public enum TargetMode {
            SHOOTING, SHUTTLING, CLIMBING
        }

        public static final double GAIN_SCHEDULE_THRESHOLD = 0.1;
        public static final double GAIN_SCHEDULE_FACTOR_P = 0.5;

        public static final double DRIVE_TO_POINT_P = 6; // tuned up for dtpfinite
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
        public static final Pose2d BLUE_CLIMB_CENTER = new Pose2d(2, 4, new Rotation2d(0));
        public static final Pose2d BLUE_CLIMB_LEFT = new Pose2d(2, 3, new Rotation2d(0));
        public static final Pose2d BLUE_CLIMB_RIGHT = new Pose2d(2, 5, new Rotation2d(0));

        public static final Pose2d RED_CLIMB_CENTER = new Pose2d(16, 4.03, new Rotation2d(0));
        public static final Pose2d RED_CLIMB_LEFT = new Pose2d(16, 4.03, new Rotation2d(0));
        public static final Pose2d RED_CLIMB_RIGHT = new Pose2d(16, 4.03, new Rotation2d(0));

        // Shuttle positions (NEED TO BE UPDATED)
        public static final Pose2d BLUE_SHUTTLE_LEFT = new Pose2d(6, 7, new Rotation2d(0));
        public static final Pose2d BLUE_SHUTTLE_RIGHT = new Pose2d(6, 1, new Rotation2d(0));

        public static final Pose2d RED_SHUTTLE_LEFT = new Pose2d(10, 1, new Rotation2d(0));
        public static final Pose2d RED_SHUTTLE_RIGHT = new Pose2d(10, 7, new Rotation2d(0));

        // Pose lists
        public static final List<Pose2d> BLUE_RADIAL_SHOOTING_POSES = List.of(BLUE_CENTER_SHOT, BLUE_RIGHT_SHOT,
                BLUE_MID_RIGHT_SHOT,
                BLUE_LEFT_SHOT, BLUE_MID_LEFT_SHOT);
        public static final List<Pose2d> RED_RADIAL_SHOOTING_POSES = List.of(RED_CENTER_SHOT, RED_RIGHT_SHOT,
                RED_MID_RIGHT_SHOT,
                RED_LEFT_SHOT, RED_MID_LEFT_SHOT);

        public static final List<Pose2d> BLUE_NEUTRAL_ZONE_POSES = List.of(BLUE_PAST_BUMP_RIGHT, BLUE_PAST_BUMP_LEFT);
        public static final List<Pose2d> RED_NEUTRAL_ZONE_POSES = List.of(RED_PAST_BUMP_RIGHT, RED_PAST_BUMP_LEFT);

        public static final List<Pose2d> BLUE_SHUTTLE_POSES = List.of(BLUE_SHUTTLE_LEFT, BLUE_SHUTTLE_RIGHT);
        public static final List<Pose2d> RED_SHUTTLE_POSES = List.of(RED_SHUTTLE_LEFT, RED_SHUTTLE_RIGHT);

        public static final List<Pose2d> BLUE_CLIMB_POSES = List.of(BLUE_CLIMB_LEFT, BLUE_CLIMB_CENTER,
                BLUE_CLIMB_RIGHT);
        public static final List<Pose2d> RED_CLIMB_POSES = List.of(RED_CLIMB_LEFT, RED_CLIMB_CENTER, RED_CLIMB_RIGHT);

        public static final Pose2d BLUE_SHOOT_TOP = new Pose2d(2.75, 5.3, new Rotation2d(-135));
        public static final Pose2d BLUE_SHOOT_BOTTOM = new Pose2d(2.75, 2.769326, new Rotation2d(-135));

        public static final Pose2d RED_SHOOT_TOP = new Pose2d(13.790988, 5.3, new Rotation2d(-135));
        public static final Pose2d RED_SHOOT_BOTTOM = new Pose2d(13.790988, 2.769326, new Rotation2d(-135));

        public static final List<Pose2d> SHOOT_POSES = List.of(BLUE_SHOOT_TOP, BLUE_SHOOT_BOTTOM, RED_SHOOT_BOTTOM,
                RED_SHOOT_TOP);

        public static final Pose2d NEUTRAL_INTAKE_TOP = new Pose2d(8.27, 7.2, new Rotation2d(-100));
        public static final Pose2d NEUTRAL_INTAKE_BOTTOM = new Pose2d(8.27, 0.869326, new Rotation2d(100));

        public static final List<Pose2d> NEUTRAL_ZONE_POSES = List.of(NEUTRAL_INTAKE_BOTTOM, NEUTRAL_INTAKE_TOP);
    }

    public static class FieldMeasurementConstants {
        public static final double ALLIANCE_ZONE_BLUE = 3.978;
        public static final double ALLIANCE_ZONE_RED = 12.563;
        public static final double RED_TRENCH_X = 12.0;
        public static final double BLUE_TRENCH_X = 4.5;
        public static final double MID_FIELD_Y = 4.034663;
        public static final double MID_FIELD_X = 8.270494;
        public static final Pose2d RED_HUB = new Pose2d(11.915394, 4.034663, new Rotation2d(0));
        public static final Pose2d BLUE_HUB = new Pose2d(4.625594, 4.034663, new Rotation2d(0));
        public static final Pose2d SHUTTLE_TARGET_TOP_RED = new Pose2d(16.5, 6, new Rotation2d(0));
        public static final Pose2d SHUTTLE_TARGET_BOTTOM_RED = new Pose2d(16.5, 2, new Rotation2d(0));
        public static final Pose2d SHUTTLE_TARGET_TOP_BLUE = new Pose2d(0, 7, new Rotation2d(0));
        public static final Pose2d SHUTTLE_TARGET_BOTTOM_BLUE = new Pose2d(0, 1, new Rotation2d(0));
        public static final Pose2d ALLIANCE_WALL_TARGET_RED = new Pose2d(16.5, 4.034663, new Rotation2d(0));
        public static final Pose2d ALLIANCE_WALL_TARGET_BLUE = new Pose2d(0.0, 4.034663, new Rotation2d(0));
        public static final double ZERO = 0;
        public static final double FIELD_DIMENSION_X = 16.540988;
        public static final double FIELD_DIMENSION_Y = 8.069326;
        public static final double BLUE_DEPOT_BUMP_NEUTRAL_X = 5.222494;
        public static final double BLUE_DEPOT_BUMP_ALLIANCE_X = 4.061714;
        public static final double RED_DEPOT_BUMP_NEUTRAL_X = 11.318494;
        public static final double RED_DEPOT_BUMP_ALLIANCE_X = 12.479274;
        public static final double DEPOT_BUMP_Y = 6.477508;
        public static final double OUTPOST_BUMP_Y = 1.53289;
        public static final double TRENCH_Y_LEFT = 7.0;
        public static final double TRENCH_Y_RIGHT = 1.0;
    }

    public static class AprilTagConstants {
        public static final List<Integer> ALL_HUB_TAGS = List.of(
                2, 3, 4, 5, 8, 9, 10, 11, 18, 19, 20, 21, 24, 25, 26, 27);
    }

    public static class AutonConstants {
        public static final double LOOP_DEGREES_ROTATED = 180;
        public static final double SHOOTING_TIME = 7;
    }
}
