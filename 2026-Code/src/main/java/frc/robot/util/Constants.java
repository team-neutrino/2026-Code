package frc.robot.util;

import static edu.wpi.first.units.Units.Meter;

import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.TreeMap;

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
        public static final double CURRENT_LIMIT = 40;
        public static final double CURRENT_HOOD_LIMIT = 4;
        public static final double SHOOTING_KP = 90.0;
        public static final double SHOOTING_KI = 0.0;
        public static final double SHOOTING_KD = 0.0;
        public static final double HOOD_KP = 300;
        public static final double HOOD_KI = 150.0;
        public static final double HOOD_KD = 0;
        public static final double ALLOWED_ERROR = 2.5;
        public static final double ALLOWED_RPM_ERROR = 80;
        public static final int SHOOTER_ID = 16;
        public static final int SHOOTER_FOLLOWER_ID = 17;
        public static final int HOOD_ID = 15;
        public static final double START_POSITION = 0;
        public static final double DEFAULT_SHOOTING_SPEED = 3000;
        public static final double CURRENT_SPIKE = 20.0;
        public static final double HOOD_GEAR_RATIO = 105.8239;
        public static final double NOT_MOVING_THRESHOLD = 0.1;
        public static final double NOT_TURNING_THRESHOLD = 10;
        public static final double SHOOTER_RPM_NOISE = 0.9;
        public static final double SHOOTER_RPM = 4000;

        // meters shooter math
        public static final double FLYWHEEL_DIAMETER = 0.1016;
        public static final double FLYWHEEL_CIRCUMFRANCE = (FLYWHEEL_DIAMETER * Math.PI);
        public static final double Y_DISPLACEMENT = 1.2376;
        public static final double GRAVITY = -9.807;

        public static enum fakeEnum { // fake temporary enum while swerve sets up fixed positions for shooter
            RADIAL_CLOSE,
            RADIAL_FAR,
            WALL,
            DEPOT,
            OUTPOST
        };

        public static final InterpolatingDoubleTreeMap CONSTANT_SPEED_HOOD = InterpolatingDoubleTreeMap.ofEntries(
                Map.entry(0.0, 6.5),
                Map.entry(1.64, 10.0),
                Map.entry(2.33, 13.5),
                Map.entry(2.68, 15.0),
                Map.entry(3.36, 30.0),
                Map.entry(4.0, 30.0));

        public static final InterpolatingDoubleTreeMap SLOW_INTERPOLATION_HOOD = InterpolatingDoubleTreeMap.ofEntries(
                Map.entry(0.0, 6.5),
                Map.entry(1.64, 10.0),
                Map.entry(2.33, 13.5),
                Map.entry(2.68, 15.0),
                Map.entry(3.36, 25.0),
                Map.entry(4.0, 25.0));

        public static final InterpolatingDoubleTreeMap FAST_INTERPOLATION_HOOD = InterpolatingDoubleTreeMap.ofEntries(
                Map.entry(0.0, 8.5),
                Map.entry(1.94, 5.0),
                Map.entry(2.46, 7.5),
                Map.entry(3.0, 9.0),
                Map.entry(3.64, 11.0),
                Map.entry(4.28, 13.0),
                Map.entry(4.92, 17.0),
                Map.entry(5.7, 21.0));

        public static TreeMap<Double, Double> SHOOTER_SPEED_ZONES = new TreeMap<Double, Double>(Map.ofEntries(
                Map.entry(0.0, 3000.0),
                Map.entry(3.5, 4000.0),
                Map.entry(4.0, 4000.0),
                Map.entry(23.9, 5800.0)));

        public static TreeMap<Double, InterpolatingDoubleTreeMap> SPEED_HOOD_INTERPOLATION = new TreeMap<Double, InterpolatingDoubleTreeMap>(
                Map.ofEntries(
                        Map.entry(0.0, SLOW_INTERPOLATION_HOOD),
                        Map.entry(3000.0, SLOW_INTERPOLATION_HOOD),
                        Map.entry(4000.0, FAST_INTERPOLATION_HOOD),
                        Map.entry(5800.0, FAST_INTERPOLATION_HOOD)));

        public static final double SHUTTLE_SHOOTING_SPEED = 6000;
        public static final double SHUTTLE_ANGLE = 25;
        public static final double MAX_SAFE_HOOD_ANGLE = 25;

        public static enum shooterConditions {
            SHOOTER_SPEED_CORRECT,
            HOOD_ANGLE_CORRECT,
            HUB_ACTIVE,
            IN_ALLIANCE_ZONE,
            NOT_DRIVING,
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
        public static final double TURRET_FF = 0.05;
        public static final double SENSOR_TO_MECHANISM_RATIO = 8.4;
        public static final double ROTOR_TO_SENSOR_RATIO = 12 / 56;
        public static final double TORQUE_LOAD = 0;
        public static final double ALLOWED_ERROR = 1;
        public static final double DISCONTINUITY_POINT = 1;
        public static final double ENCODER_MAGNET_OFFSET = 0;
        public static final double STATIC_FF = 0;
        public static final double VELOCITY_FF = 0;
        public static final double ACCELERATION_FF = 0;
        public static final double MAX_WINDUP = 360;
        public static final double MIN_WINDUP = -180;
        public static final double TARGET_TOLERANCE = 1;
        public static final double TURRET_OFFSET_X = -0.127;
        public static final double TURRET_OFFSET_Y = -0.1016;
    }

    public static class IndexerConstants {
        public static final int SPINDEXER_MOTOR_ID = 14;

        public static final double INDEXING_VOLTAGE = -10;
        public static final double CURRENT_LIMIT = 40;
    }

    public static class IntakeConstants {
        public static final int ROLLER_MOTOR_ID = 12;
        public static final int DEPLOY_MOTOR_ID = 13;

        public static final double CURRENT_LIMIT = 60;

        public static final double INTAKE_VOLTAGE = -12;
        public static final double OUTTAKE_VOLTAGE = 12;

        public static final double STARTING_POSITION = -.50;
        public static final double DEPLOYED_POSITION = -8.5;
        public static final double INTERMEDIATE_POSITION_1 = 60;
        public static final double INTERMEDIATE_POSITION_2 = 30;
        public static final double ALLOWED_TARGET_ERROR = 4;
        public static final double ALLOWED_INTAKE_ERROR = 4;

        public static final double INTAKE_kP = 2;
        public static final double INTAKE_kI = 0.0;
        public static final double INTAKE_kD = 0.0;
    }

    public static class KickerConstants {
        public static final int KICKER_MOTOR_ID = 18;

        public static final double KICKER_CURRENT_LIMIT = 40;
        public static final double KICKER_VOLTAGE = 8;
    }

    public static class LimelightConstants {
        public static final String LL_FRONT = "limelight-front";
        public static final String LL_BACK = "limelight-back";
        public static final String LL_LEFT = "limelight-left";
        public static final String LL_RIGHT = "limelight-right";

        // Camera pose offsets in meters
        public static final double FRONT_FORWARD_OFFSET = 0.0325374;
        public static final double FRONT_SIDE_OFFSET = -0.047625;
        public static final double FRONT_HEIGHT_OFFSET = 0.7458202;
        public static final double FRONT_ROLL_OFFSET = 0.0;
        public static final double FRONT_PITCH_OFFSET = 27.5;
        public static final double FRONT_YAW_OFFSET = 0.0;

        public static final double BACK_FORWARD_OFFSET = -0.3310382;
        public static final double BACK_SIDE_OFFSET = 0.17145; // figure out if left or right
        public static final double BACK_HEIGHT_OFFSET = 0.3654298;
        public static final double BACK_ROLL_OFFSET = 0.0;
        public static final double BACK_PITCH_OFFSET = 20;
        public static final double BACK_YAW_OFFSET = 180;

        public static final double LEFT_FORWARD_OFFSET = -0.260255;
        public static final double LEFT_SIDE_OFFSET = -0.308226;
        public static final double LEFT_HEIGHT_OFFSET = 0.208553;
        public static final double LEFT_ROLL_OFFSET = 0; // Roll (degrees)
        public static final double LEFT_PITCH_OFFSET = 31; // Pitch (degrees)
        public static final double LEFT_YAW_OFFSET = 74; // Yaw (degrees)

        public static final double RIGHT_FORWARD_OFFSET = -0.260255; // Forward offset (meters)
        public static final double RIGHT_SIDE_OFFSET = 0.308226; // Side offset (meters)
        public static final double RIGHT_HEIGHT_OFFSET = 0.208553; // Height offset (meters)
        public static final double RIGHT_ROLL_OFFSET = 0; // Roll (degrees)
        public static final double RIGHT_PITCH_OFFSET = 31; // Pitch (degrees)
        public static final double RIGHT_YAW_OFFSET = 286; // Yaw (degrees)

        public static final double MINIMUM_XY_STD_DEV_LL4 = 1.1;
        public static final double MINIMUM_THETA_STD_DEV_LL4 = 0.2;
        public static final double MINIMUM_XY_STD_DEV_LL3G = 1.5;
        public static final double MINIMUM_THETA_STD_DEV_LL3G = 0.4;
        public static final double MINIMUM_XY_STD_DEV_LL3 = 1.8;
        public static final double MINIMUM_THETA_STD_DEV_LL3 = 0.6;
        public static final double ERROR_FACTOR_LL4 = 0.5;
        public static final double ERROR_FACTOR_LL3 = 0.9;
        public static final double ERROR_FACTOR_LL3G = 0.7;

        public static final double BUMP_MINIMUM_THRESHOLD = 7;
        public static final double IGNORE_MEASUREMENT_STD_DEV = 999999999;
        public static final double MT1_WEIGHT_YAW = .7;
        public static final double PIGEON_SEED_PERIOD = 5;
        public static final double EXTERNAL_WEIGHT = 0.01; // larger number (0-1) = lower trust in internal IMU
    }

    public static class SwerveConstants {
        public static final double GYRO_SCALAR_Z = -3.9;
        public static final double MAX_SPEED = 5.7;
        public static final double MAX_ROTATION_SPEED = 1.5 * Math.PI;
        public static final double ROTATIONAL_P = 6;
        public static final double AUTO_ALIGN_D = 0;
        public static final double JOYSTICK_REST_ALLOWED_ERROR = 0.1;
    }

    public static class DriveToPointConstants {
        public enum TargetMode {
            SHOOTING, SHUTTLING, CLIMBING
        }

        public static final double GAIN_SCHEDULE_THRESHOLD = 0.1;
        public static final double GAIN_SCHEDULE_FACTOR_P = 0.5;

        public static final double DRIVE_TO_POINT_P = 4;
        public static final double DRIVE_TO_POINT_I = 0;
        public static final double DRIVE_TO_POINT_D = 0.5;
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

    }

    public static class FieldMeasurementConstants {
        public static final double ALLIANCE_ZONE_BLUE = 3.978;
        public static final double ALLIANCE_ZONE_RED = 12.563;
        public static final double MID_FIELD = 4.034663;
        public static final Pose2d RED_HUB = new Pose2d(11.915394, 4.034663, new Rotation2d(0));
        public static final Pose2d BLUE_HUB = new Pose2d(4.625594, 4.034663, new Rotation2d(0));
        public static final Pose2d SHUTTLE_TARGET_TOP_RED = new Pose2d(16.5, 7, new Rotation2d(0));
        public static final Pose2d SHUTTLE_TARGET_BOTTOM_RED = new Pose2d(16.5, 1, new Rotation2d(0));
        public static final Pose2d SHUTTLE_TARGET_TOP_BLUE = new Pose2d(0, 7, new Rotation2d(0));
        public static final Pose2d SHUTTLE_TARGET_BOTTOM_BLUE = new Pose2d(0, 1, new Rotation2d(0));
        public static final Distance ZERO = Distance.ofBaseUnits(0, Meter);
        public static final Distance FIELD_DIMENSION_X = Distance.ofBaseUnits(Units.inchesToMeters(650.12), Meter);
        public static final Distance FIELD_DIMENSION_Y = Distance.ofBaseUnits(Units.inchesToMeters(316.64), Meter);
    }
}
