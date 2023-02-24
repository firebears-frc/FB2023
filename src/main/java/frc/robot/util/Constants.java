package frc.robot.util;

public class Constants {
    public static class ChassisConstants {
        public static final int RIGHT_FRONT_PORT = 15;
        public static final int RIGHT_BACK_PORT = 17;
        public static final int LEFT_FRONT_PORT = 16;
        public static final int LEFT_BACK_PORT = 18;

        public static final int STALL_CURRENT_LIMIT = 30;
        public static final int FREE_CURRENT_LIMIT = 20;
        public static final double SECONDARY_CURRENT_LIMIT = 60.0;

        public static final double TRACK_WIDTH = 0.96679; // Meters
        public static final double GEAR_RATIO = (52.0 / 10.0) * (68.0 / 30.0);
        public static final double WHEEL_DIAMETER = 0.2032; // 8 inches
        public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
        public static final double METERS_PER_MOTOR_ROTATION = WHEEL_CIRCUMFERENCE / GEAR_RATIO;
        public static final double VELOCITY_CONVERSION_FACTOR = METERS_PER_MOTOR_ROTATION / 60; // The raw units are RPM

        public static final double MAX_VELOCITY = 5; // Meters per second
        public static final double MAX_ANGULAR_VELOCITY = 8; // Radians per second
        public static final double MAX_ACCELERATION = 5; // Meters per second squared: TODO!
        public static final double MAX_VOLTAGE = 10;

        // Values spit out of sysid
        public static final double P = 0.011179;
        public static final double I = 0;
        public static final double D = 0;
        public static final double S = 0.15473;
        public static final double V = 2.3007;
        public static final double A = 0.22029;
    }

    public static class ArmConstants {
        public static final int SHOULDER_1_PORT = 1;
        public static final int SHOULDER_2_PORT = 1;
        public static final int ELBOW_PORT = 1;

        public static final int STALL_CURRENT_LIMIT = 30;
        public static final int FREE_CURRENT_LIMIT = 20;
        public static final double SECONDARY_CURRENT_LIMIT = 60.0;

        public static final double SHOULDER_P = 0.0001;
        public static final double SHOULDER_I = 0;
        public static final double SHOULDER_D = 0;
        public static final double SHOULDER_OFFSET = 0; // degrees

        public static final double ELBOW_P = 0.0001;
        public static final double ELBOW_I = 0;
        public static final double ELBOW_D = 0;
        public static final double ELBOW_OFFSET = 0; // degrees
    }
}
