package frc.robot;

import edu.wpi.first.wpilibj.Preferences;

import static frc.robot.util.Config.cleanAllPreferences;
import static frc.robot.util.Config.loadConfiguration;
import static frc.robot.util.Config.printPreferences;

public class Constants {
    /** DEBUG enables extra logging and Shuffleboard widgets. */
    public static boolean DEBUG = false;

    public static boolean PRACTICE_ROBOT = true;

    public static double ELBOW_ENCODER_OFFSET;
    public static double SHOULDER_ENCODER_OFFSET;

    public static final class DriveConstants {
        public static final int kLeftMotor1Port = 16;
        public static final int kLeftMotor2Port = 18;
        public static final int kRightMotor1Port = 15;
        public static final int kRightMotor2Port = 17;

        public static final float kP = 0.011179f;
        public static final float kI = 0;
        public static final float kD = 0;

        public static final float kS = 0.16084f;
        public static final float kV = 2.2918f;
        public static final float kA = 0.36793f;

        public static final double TrackWidth = 0.96679;

        public static final float MaxVelocity = 5; // m/s
        public static final float MaxAngularVelocity = 8; // radians per second
    }

    public static float kFeetToMeterFactor = 0.638f / 11.79f;
    public static int shoulderArmLength=30;
    public static int elbowArmLength=33;

    public static void init(String... fileNames) {
        cleanAllPreferences();
        loadConfiguration(fileNames);
        printPreferences(System.out);

        // The following values should be pulled from a config.properties file:
        DEBUG = Preferences.getBoolean("DEBUG", false);
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
