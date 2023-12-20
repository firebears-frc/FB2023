package frc.robot.util.sparkmax;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

public interface StatusFrameConfiguration {
    // https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
    public void apply(CANSparkMax motor);

    public static StatusFrameConfiguration normal() {
        return new StatusFrameConfiguration() {
            private static final int[] NORMAL_FRAME_CONFIGURATION = { 20, 20, 20, 1000, 1000, 1000, 1000 };

            @Override
            public void apply(CANSparkMax motor) {
                StatusFrameConfiguration.apply(motor, NORMAL_FRAME_CONFIGURATION);
            }
        };
    }

    public static StatusFrameConfiguration absoluteEncoder() {
        return new StatusFrameConfiguration() {
            private static final int[] ABSOLUTE_ENCODER_CONFIGURATION = { 20, 20, 20, 1000, 1000, 20, 1000 };

            @Override
            public void apply(CANSparkMax motor) {
                StatusFrameConfiguration.apply(motor, ABSOLUTE_ENCODER_CONFIGURATION);
            }
        };
    }

    public static StatusFrameConfiguration leadingAbsoluteEncoder() {
        return new StatusFrameConfiguration() {
            private static final int[] LEADING_ABSOLUTE_ENCODER_CONFIGURATION = { 1, 20, 20, 1000, 1000, 20, 1000 };

            @Override
            public void apply(CANSparkMax motor) {
                StatusFrameConfiguration.apply(motor, LEADING_ABSOLUTE_ENCODER_CONFIGURATION);
            }
        };
    }

    private static void apply(CANSparkMax motor, int periods[]) {
        if (periods.length != PeriodicFrame.values().length)
            throw new IllegalArgumentException(
                    "Status frame periods must be " + PeriodicFrame.values().length + " long, not " + periods.length);

        for (int i = 0; i < periods.length; i++) {
            PeriodicFrame frame = PeriodicFrame.fromId(i);
            Util.configure(period -> motor.setPeriodicFramePeriod(frame, period), periods[i], frame.name());
        }
    }
}
