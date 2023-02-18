package frc.robot;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.util.Config.cleanAllPreferences;
import static frc.robot.util.Config.loadConfiguration;
import static frc.robot.util.Config.printPreferences;

public class Constants {
    /** DEBUG enables extra logging and Shuffleboard widgets. */
    public static boolean DEBUG = false;

    public static boolean PRACTICE_ROBOT = true;

    public static final class DriveConstants {
        public static final int kLeftMotor1Port = 16;
        public static final int kLeftMotor2Port = 18;
        public static final int kRightMotor1Port = 15;
        public static final int kRightMotor2Port = 17;
    }

    public static int kFeetToMeterFactor = 25;

    public static void init(String... fileNames) {
        cleanAllPreferences();
        loadConfiguration(fileNames);
        printPreferences(System.out);

        DEBUG = Preferences.getBoolean("DEBUG", false);
        PRACTICE_ROBOT = Preferences.getBoolean("PRACTICE_ROBOT", true);

        if (PRACTICE_ROBOT) {
            kFeetToMeterFactor = 25;
        } else { 
            // Competition bot
            kFeetToMeterFactor = 20;
            
        }
        System.out.println("kFeetToMeterFactor = " + kFeetToMeterFactor);
    }
}
