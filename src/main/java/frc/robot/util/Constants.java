package frc.robot.util;

public class Constants {
    public static final int JOYSTICK_PORT = 0;
    public static final int CONTROLLER_PORT = 1;

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
        public static final int ELBOW_PORT = 7;
        public static final int ELBOW_STALL_CURRENT_LIMIT = 35;
        public static final int ELBOW_FREE_CURRENT_LIMIT = 20;
        public static final double ELBOW_SECONDARY_CURRENT_LIMIT = 40.0;
        public static final double ELBOW_SPEED = 2.0; // degrees per loop
        public static final double ELBOW_P = 0.01;
        public static final double ELBOW_I = 0;
        public static final double ELBOW_D = 0.0005;
        public static final double ELBOW_OFFSET = 240; // degrees
        public static final double ELBOW_SUBSTATION = 0; // degrees
        public static final double ELBOW_STOW = 0; // degrees
        public static final double ELBOW_HIGH = 0; // degrees
        public static final double ELBOW_MID = 0; // degrees
        public static final double ELBOW_LOW = 0; // degrees

        public static final int SHOULDER_RIGHT_PORT = 12;
        public static final int SHOULDER_LEFT_PORT = 13;
        public static final int SHOULDER_STALL_CURRENT_LIMIT = 35;
        public static final int SHOULDER_FREE_CURRENT_LIMIT = 20;
        public static final double SHOULDER_SECONDARY_CURRENT_LIMIT = 40.0;
        public static final double SHOULDER_SPEED = 2.0; // degrees per loop
        public static final double SHOULDER_P = 0.0001;
        public static final double SHOULDER_I = 0;
        public static final double SHOULDER_D = 0.0005;
        public static final double SHOULDER_OFFSET = 196; // degrees
        public static final double SHOULDER_SUBSTATION = 0; // degrees
        public static final double SHOULDER_STOW = 0; // degrees
        public static final double SHOULDER_HIGH = 0; // degrees
        public static final double SHOULDER_MID = 0; // degrees
        public static final double SHOULDER_LOW = 0; // degrees

    }

    public static class SchluckerConstants {
        public static final int MOTOR_PORT = 6;

        public static final int STALL_CURRENT_LIMIT = 20;
        public static final int FREE_CURRENT_LIMIT = 5;
        public static final double SECONDARY_CURRENT_LIMIT = 30.0;

        public static final double CONE_IN_CUBE_OUT_SPEED = 0.7;
        public static final double CONE_OUT_CUBE_IN_SPEED = -0.7;
    }
}
