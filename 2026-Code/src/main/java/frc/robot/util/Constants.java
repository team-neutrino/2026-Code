package frc.robot.util;

public class Constants {
    public static class ShooterConstants {
        public static final double SHOOTING_VOLTAGE = -12;
        public static final double CURRENT_LIMIT = 40;
    }

    public static class IndexerConstants {
        public static final int MOTOR_1_ID = 14;
        public static final int MOTOR_2_ID = 15;
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
}
