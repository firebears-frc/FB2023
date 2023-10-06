package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ChargeStationStatus;
import frc.robot.util.GamePiece;

public class Lights extends SubsystemBase {
    private final Supplier<GamePiece> itemHeldSupplier;
    private final Supplier<GamePiece> itemWantedSupplier;
    private final Supplier<Boolean> levelSupplier;
    private final Supplier<Boolean> onChargeStationSupplier;
    private final Supplier<Boolean> isNotPitchingSupplier;
    private final ParallelBus communication;
    private ChargeStationStatus chargeStationStatus;

    public Lights(Supplier<GamePiece> itemHeldSupplier, Supplier<GamePiece> itemWantedSupplier,
            Supplier<Boolean> levelSupplier, Supplier<Boolean> onChargeStationSupplier,
            Supplier<Boolean> isPitchingSupplier) {
        this.itemHeldSupplier = itemHeldSupplier;
        this.itemWantedSupplier = itemWantedSupplier;
        this.levelSupplier = levelSupplier;
        this.onChargeStationSupplier = onChargeStationSupplier;
        this.isNotPitchingSupplier = isPitchingSupplier;
        communication = new ParallelBus();
        chargeStationStatus = ChargeStationStatus.NONE;
    }

    private enum Status {
        DISABLED,
        DISABLED_RED,
        DISABLED_BLUE,

        AUTO_NONE,
        AUTO_HAS_CUBE,
        AUTO_HAS_CONE,
        AUTO_DOCKED,
        AUTO_DOCKED_RED,
        AUTO_DOCKED_BLUE,
        AUTO_ENGAGED,
        AUTO_ENGAGED_RED,
        AUTO_ENGAGED_BLUE,

        TELEOP_NONE,
        TELEOP_HAS_CUBE,
        TELEOP_HAS_CONE,
        TELEOP_WANT_CUBE,
        TELEOP_WANT_CONE,
        TELEOP_DOCKED,
        TELEOP_DOCKED_RED,
        TELEOP_DOCKED_BLUE,
        TELEOP_ENGAGED,
        TELEOP_ENGAGED_RED,
        TELEOP_ENGAGED_BLUE
    }

    private Status getAlliance(Status red, Status blue, Status none) {
        if (!DriverStation.isFMSAttached())
            return none;

        switch (DriverStation.getAlliance()) {
            case Red:
                return red;
            case Blue:
                return blue;
            case Invalid:
            default:
                return none;
        }
    }

    private void updateChargeStationStatus() {
        if (levelSupplier == null || onChargeStationSupplier == null || isNotPitchingSupplier == null) {
            chargeStationStatus = ChargeStationStatus.NONE;
            return;
        }

        switch (chargeStationStatus) {
            case ENGAGED:
                if (!levelSupplier.get()) {
                    chargeStationStatus = ChargeStationStatus.DOCKED;
                }
                break;
            case DOCKED:
                if (levelSupplier.get() && isNotPitchingSupplier.get()) {
                    chargeStationStatus = ChargeStationStatus.ENGAGED;
                }
                break;
            case NONE:
            default:
                if (onChargeStationSupplier.get()) {
                    chargeStationStatus = ChargeStationStatus.DOCKED;
                }
                break;
        }
    }

    private Status getStatus() {
        // Disabled
        if (!DriverStation.isEnabled()) {
            chargeStationStatus = ChargeStationStatus.NONE;
            return getAlliance(Status.DISABLED_RED, Status.DISABLED_BLUE, Status.DISABLED);
        }

        updateChargeStationStatus();

        // Auto
        if (DriverStation.isAutonomousEnabled()) {
            switch (chargeStationStatus) {
                case ENGAGED:
                    return getAlliance(Status.AUTO_ENGAGED_RED, Status.AUTO_ENGAGED_BLUE, Status.AUTO_ENGAGED);
                case DOCKED:
                    return getAlliance(Status.AUTO_DOCKED_RED, Status.AUTO_DOCKED_BLUE, Status.AUTO_DOCKED);
                case NONE:
                default:
                    break;
            }
            if (itemHeldSupplier != null) {
                switch (itemHeldSupplier.get()) {
                    case CONE:
                        return Status.AUTO_HAS_CONE;
                    case CUBE:
                        return Status.AUTO_HAS_CUBE;
                    case NONE:
                    default:
                        break;
                }
            }
            return Status.AUTO_NONE;
        }

        // Teleop
        if (DriverStation.isTeleopEnabled()) {
            switch (chargeStationStatus) {
                case ENGAGED:
                    return getAlliance(Status.TELEOP_ENGAGED_RED, Status.TELEOP_ENGAGED_BLUE,
                            Status.TELEOP_ENGAGED);
                case DOCKED:
                    return getAlliance(Status.TELEOP_DOCKED_RED, Status.TELEOP_DOCKED_BLUE, Status.TELEOP_DOCKED);
                case NONE:
                default:
                    break;
            }
            if (itemHeldSupplier != null) {
                switch (itemHeldSupplier.get()) {
                    case CONE:
                        return Status.TELEOP_HAS_CONE;
                    case CUBE:
                        return Status.TELEOP_HAS_CUBE;
                    case NONE:
                    default:
                        break;
                }
            }
            if (itemWantedSupplier != null) {
                switch (itemWantedSupplier.get()) {
                    case CONE:
                        return Status.TELEOP_WANT_CONE;
                    case CUBE:
                        return Status.TELEOP_WANT_CUBE;
                    case NONE:
                    default:
                        break;
                }
            }

            return Status.TELEOP_NONE;
        }

        // Something went wrong...
        return Status.DISABLED;
    }

    @Override
    public void periodic() {
        communication.set(getStatus().ordinal());
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
