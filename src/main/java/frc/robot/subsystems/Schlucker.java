package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.GamePiece;

public class Schlucker extends SubsystemBase {
    public static class Constants {
        public static final int MOTOR_PORT = 6;
    }

    protected enum SchluckerState {
        INTAKE,
        EJECT,
        HOLD,
        STOP
    }

    protected SchluckerState state = SchluckerState.STOP;
    protected GamePiece itemHeld = GamePiece.NONE;
    protected GamePiece lastItemHeld = GamePiece.NONE;
    protected GamePiece itemWanted = GamePiece.NONE;

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
}
