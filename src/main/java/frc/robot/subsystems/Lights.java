package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.GamePiece;

public class Lights extends SubsystemBase {
    private final Supplier<GamePiece> itemSupplier;
    private final ParallelBus communication;

    public Lights(Supplier<GamePiece> itemSupplier) {
        this.itemSupplier = itemSupplier;
        communication = new ParallelBus();
    }

    private Status getStatus() {
        // Disabled
        if (!DriverStation.isEnabled()) {
            if (!DriverStation.isFMSAttached())
                return Status.DISABLED;

            switch (DriverStation.getAlliance()) {
                case Red:
                    return Status.DISABLED_RED;
                case Blue:
                    return Status.DISABLED_BLUE;
                case Invalid:
                default:
                    return Status.DISABLED;
            }
        }

        // Auto
        if (DriverStation.isAutonomous()) {
            if (itemSupplier != null) {
                switch (itemSupplier.get()) {
                    case CONE:
                        return Status.AUTO_HAS_CONE;
                    case CUBE:
                        return Status.AUTO_HAS_CUBE;
                    case NONE:
                    default:
                        return Status.AUTO_NONE;
                }
            }
            return Status.AUTO_NONE;
        }
    }

    @Override
    public void periodic() {
        communication.set(getStatus().ordinal());
    }

    private enum Status {
        DISABLED,
        DISABLED_RED,
        DISABLED_BLUE,

        AUTO_NONE,
        AUTO_HAS_CUBE,
        AUTO_HAS_CONE,
        AUTO_BALANCED_RED,
        AUTO_BALANCED_BLUE,

        TELEOP_HAS_CUBE,
        TELEOP_HAS_CONE,
        TELEOP_WANT_CUBE,
        TELEOP_WANT_CONE,
        TELEOP_BALANCED_RED,
        TELEOP_BALANCED_BLUE
    }

    private static class ParallelBus {
        private final DigitalOutput outputA;
        private final DigitalOutput outputB;
        private final DigitalOutput outputC;
        private final DigitalOutput outputD;
        private final DigitalOutput outputE;

        public ParallelBus() {
            outputA = new DigitalOutput(10);
            outputB = new DigitalOutput(11);
            outputC = new DigitalOutput(12);
            outputD = new DigitalOutput(13);
            outputE = new DigitalOutput(14);
        }

        public void set(int tx) {
            tx--;
            outputA.set((tx & 0x01) == 0x01);
            outputB.set((tx & 0x02) == 0x02);
            outputC.set((tx & 0x04) == 0x04);
            outputD.set((tx & 0x08) == 0x08);
            outputE.set((tx & 0x10) == 0x10);
        }
    }
}
