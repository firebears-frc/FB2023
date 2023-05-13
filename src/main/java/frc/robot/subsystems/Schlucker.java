package frc.robot.subsystems;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.GamePiece;

public class Schlucker extends SubsystemBase {
    public static class Constants {
        public static final int MOTOR_CAN_ID = 6;
    }

    protected enum SchluckerState {
        INTAKE,
        EJECT,
        HOLD,
        STOP
    }

    protected final StringLogEntry typeLog;
    protected final StringLogEntry stateLog;
    protected final StringLogEntry itemHeldLog;
    protected final StringLogEntry lastItemHeldLog;
    protected final StringLogEntry itemWantedLog;

    protected SchluckerState state = SchluckerState.STOP;
    protected GamePiece itemHeld = GamePiece.NONE;
    protected GamePiece lastItemHeld = GamePiece.NONE;
    protected GamePiece itemWanted = GamePiece.NONE;

    public Schlucker(DataLog log) {
        typeLog = new StringLogEntry(log, "Schlucker/Type");
        typeLog.append("Schlucker");

        stateLog = new StringLogEntry(log, "Schlucker/State");
        itemHeldLog = new StringLogEntry(log, "Schlucker/ItemHeld");
        lastItemHeldLog = new StringLogEntry(log, "Schlucker/LastItemHeld");
        itemWantedLog = new StringLogEntry(log, "Schlucker/ItemWanted");
    }

    public Command intakeCone() {
        return new InstantCommand(() -> {
            state = SchluckerState.INTAKE;
            itemHeld = GamePiece.CONE;
            lastItemHeld = GamePiece.CONE;
            itemWanted = GamePiece.NONE;
        }, this);
    }

    public Command intakeCube() {
        return new InstantCommand(() -> {
            state = SchluckerState.INTAKE;
            itemHeld = GamePiece.CUBE;
            lastItemHeld = GamePiece.CUBE;
            itemWanted = GamePiece.NONE;
        }, this);
    }

    public Command hold() {
        return new InstantCommand(() -> {
            state = SchluckerState.HOLD;
        }, this);
    }

    public Command eject() {
        return new InstantCommand(() -> {
            state = SchluckerState.EJECT;
            itemHeld = GamePiece.NONE;
        }, this);
    }

    public Command stop() {
        return new InstantCommand(() -> {
            state = SchluckerState.STOP;
        }, this);
    }

    public Command wantCone() {
        return new InstantCommand(() -> {
            itemWanted = GamePiece.CONE;
        }, this);
    }

    public Command wantCube() {
        return new InstantCommand(() -> {
            itemWanted = GamePiece.CUBE;
        }, this);
    }

    public GamePiece getHeldItem() {
        return itemHeld;
    }

    public GamePiece getWantedItem() {
        return itemWanted;
    }

    @Override
    public void periodic() {
        stateLog.append(state.name());
        itemHeldLog.append(itemHeld.name());
        lastItemHeldLog.append(lastItemHeld.name());
        itemWantedLog.append(itemWanted.name());
    }
}
