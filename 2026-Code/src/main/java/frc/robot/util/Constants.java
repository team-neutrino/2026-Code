package frc.robot.util;

import java.util.Optional;

import com.ctre.phoenix6.CANBus;

public class Constants {
    public static class RioConstants {
        public static final CANBus RIO_BUS = new CANBus("rio");
    }

    public static class GlobalConstants {
        public static Optional<Boolean> RED_ALLIANCE = Optional.empty();
    }

    public static class SwerveConstants {
        public static final double GYRO_SCALAR_Z = -3.9;
        public static final double MAX_SPEED = 5.7;
        public static final double MAX_ROTATION_SPEED = 1.5 * Math.PI;
        public static final double ROTATIONAL_P = 6;
        public static final double AUTO_ALIGN_D = 0;
        public static final double JOYSTICK_REST_ALLOWED_ERROR = 0.1;
    }
}
