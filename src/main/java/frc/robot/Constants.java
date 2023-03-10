package frc.robot;

import edu.wpi.first.wpilibj.Preferences;

import static frc.robot.util.Config.cleanAllPreferences;
import static frc.robot.util.Config.loadConfiguration;
import static frc.robot.util.Config.printPreferences;

public class Constants {
    /** DEBUG enables extra logging and Shuffleboard widgets. */
    public static boolean DEBUG = false;
    public static boolean LOGGING = false;
    public static String LOG_DIR = "";

    public static boolean PRACTICE_ROBOT = true;

    public static double ELBOW_ENCODER_OFFSET;
    public static double SHOULDER_ENCODER_OFFSET;

    public static final class DriveConstants {
        public static final int kLeftMotor1Port = 16;
        public static final int kLeftMotor2Port = 18;
        public static final int kRightMotor1Port = 15;
        public static final int kRightMotor2Port = 17;
    }

    public static int kFeetToMeterFactor = 25;
    public static int shoulderArmLength = 30;
    public static int elbowArmLength = 33;

    public static void init(String... fileNames) {
        cleanAllPreferences();
        loadConfiguration(fileNames);
        printPreferences(System.out);

        // The following values should be pulled from a config.properties file:
        DEBUG = Preferences.getBoolean("DEBUG", false);
        LOGGING = Preferences.getBoolean("LOGGING", false);
        LOG_DIR = Preferences.getString("LOG_DIR", "");
        PRACTICE_ROBOT = Preferences.getBoolean("PRACTICE_ROBOT", true);
        ELBOW_ENCODER_OFFSET = Preferences.getDouble("ELBOW_ENCODER_OFFSET", 0.0);
        SHOULDER_ENCODER_OFFSET = Preferences.getDouble("SHOULDER_ENCODER_OFFSET", 0.0);

        if (PRACTICE_ROBOT) {
            kFeetToMeterFactor = 20;
        } else {
            // Competition bot
            kFeetToMeterFactor = 20;

        }
    }
}
