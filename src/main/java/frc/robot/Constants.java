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
        public static final int kRightMotor1Port = 17;
        public static final int kRightMotor2Port = 15;

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

    public static final class PracticeArmConstants {
        public static final double shoulderP = 0.0175;
        public static final double shoulderI = 0.0;
        public static final double shoulderD = 0.005;

        public static final double elbowP = 0.02;
        public static final double elbowI = 0.0;
        public static final double elbowD = 0.001;
    }

    public static final class CompArmConstants {
        public static final double shoulderP = 0.0175;
        public static final double shoulderI = 0.0;
        public static final double shoulderD = 0.005;

        public static final double elbowP = 0.02;
        public static final double elbowI = 0.0;
        public static final double elbowD = 0.005;
    }

    public static final double kFeetToMeterFactor = 0.638f / 11.79f;
    public static int ARM_SHOULDER_LENGTH = 28;
    public static int ARM_ELBOW_LENGTH = 32;

    public static final double PRACTICE_SCHLUCKER_HOLD_PERCENT = 0.15;
    public static final double COMP_SCHLUCKER_HOLD_PERCENT = 0.3;

    public static void init(String... fileNames) {
        cleanAllPreferences();
        loadConfiguration(fileNames);
        printPreferences(System.out);

        // The following values should be pulled from a config.properties file:
        DEBUG = Preferences.getBoolean("DEBUG", false);
        PRACTICE_ROBOT = Preferences.getBoolean("PRACTICE_ROBOT", true);
        ELBOW_ENCODER_OFFSET = Preferences.getDouble("ELBOW_ENCODER_OFFSET", 0.0);
        SHOULDER_ENCODER_OFFSET = Preferences.getDouble("SHOULDER_ENCODER_OFFSET", 0.0);

        /*
        if (PRACTICE_ROBOT) {
            kFeetToMeterFactor = 20;
        } else { 
            // Competition bot
            kFeetToMeterFactor = 20;
            
        }
        */
    }
}
