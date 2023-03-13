package frc.robot.util;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;

import static frc.robot.Constants.LOGGING;

/**
 * Help log values into {@link DataLogManager}, but only append values if they
 * have changed.
 */
public class LogEntryDouble extends DoubleLogEntry {

    private double previousValue = Integer.MIN_VALUE;

    public LogEntryDouble(DataLog log, String name) {
        super(log, name);
    }

    @Override
    public void append(double value, long timestamp) {
        if (LOGGING) {
            if (value != previousValue) {
                super.append(value, timestamp);
                previousValue = value;
            }
        }
    }

    @Override
    public void append(double value) {
        if (LOGGING) {
            if (value != previousValue) {
                super.append(value);
                previousValue = value;
            }
        }
    }
}
