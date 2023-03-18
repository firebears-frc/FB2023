package frc.robot;

import edu.wpi.first.wpilibj.Preferences;

import static frc.robot.util.Config.cleanAllPreferences;
import static frc.robot.util.Config.loadConfiguration;
import static frc.robot.util.Config.printPreferences;

public class Constants {
    /** DEBUG enables extra logging and Shuffleboard widgets. */
    public static boolean DEBUG = false;
    public static boolean LOGGING = false;
    public static boolean LOGGING_NT = false;
    public static boolean LOGGING_DS = true;
    public static String LOG_DIR = "";

    public static boolean PRACTICE_ROBOT = true;

    public static double ELBOW_ENCODER_OFFSET;
    public static double SHOULDER_ENCODER_OFFSET;

    public static final class DriveConstants {
        public static final int kLeftMotor1Port = 16;
        public static final int kLeftMotor2Port = 18;
        public static final int kRightMotor1Port = 15;
        public static final int kRightMotor2Port = 17;

        public static final double kP = 0.02d;//0.011179d;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double kS = 0.15473d;
        public static final double kV = 2.3007d;
        public static final double kA = 0.22029d;

        public static final double TrackWidth = 0.96679d;

        public static final double MaxVelocity = 5; // m/s
        public static final double MaxAngularVelocity = 8; // radians per second
    }

    public static final double kFeetToMeterFactor = 0.638f / 11.79f;
    public static int shoulderArmLength=30;
    public static int elbowArmLength=33;

    public static void init(String... fileNames) {
        cleanAllPreferences();
        loadConfiguration(fileNames);
        printPreferences(System.out);

        // The following values should be pulled from a config.properties file:
        DEBUG = Preferences.getBoolean("DEBUG", false);
        LOGGING = Preferences.getBoolean("LOGGING", false);
        LOG_DIR = Preferences.getString("LOG_DIR", "");
        LOGGING_NT = Preferences.getBoolean("LOGGING_NT", false);
        LOGGING_DS = Preferences.getBoolean("LOGGING_DS", true);
        PRACTICE_ROBOT = Preferences.getBoolean("PRACTICE_ROBOT", true);
        ELBOW_ENCODER_OFFSET = Preferences.getDouble("ELBOW_ENCODER_OFFSET", 0.0);
        SHOULDER_ENCODER_OFFSET = Preferences.getDouble("SHOULDER_ENCODER_OFFSET", 0.0);
    }
}
