package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
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

    protected SchluckerState state = SchluckerState.STOP;
    protected GamePiece itemHeld = GamePiece.NONE;
    protected GamePiece lastItemHeld = GamePiece.NONE;
    protected GamePiece itemWanted = GamePiece.NONE;

    public Schlucker() {
        Logger.recordMetadata("Schlucker/Type", "Schlucker");
    }

    public Command intakeCone() {
        return runOnce(() -> {
            state = SchluckerState.INTAKE;
            itemHeld = GamePiece.CONE;
            lastItemHeld = GamePiece.CONE;
            itemWanted = GamePiece.NONE;
        });
    }

    public Command intakeCube() {
        return runOnce(() -> {
            state = SchluckerState.INTAKE;
            itemHeld = GamePiece.CUBE;
            lastItemHeld = GamePiece.CUBE;
            itemWanted = GamePiece.NONE;
        });
    }

    public Command hold() {
        return runOnce(() -> {
            state = SchluckerState.HOLD;
        });
    }

    public Command eject() {
        return runOnce(() -> {
            state = SchluckerState.EJECT;
            itemHeld = GamePiece.NONE;
        });
    }

    public Command stop() {
        return runOnce(() -> {
            state = SchluckerState.STOP;
        });
    }

    public Command wantCone() {
        return runOnce(() -> {
            itemWanted = GamePiece.CONE;
        });
    }

    public Command wantCube() {
        return runOnce(() -> {
            itemWanted = GamePiece.CUBE;
        });
    }

    public GamePiece getHeldItem() {
        return itemHeld;
    }

    public GamePiece getWantedItem() {
        return itemWanted;
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Schlucker/State", state.name());
        Logger.recordOutput("Schlucker/ItemHeld", itemHeld.name());
        Logger.recordOutput("Schlucker/LastItemHeld", lastItemHeld.name());
        Logger.recordOutput("Schlucker/ItemWanted", itemWanted.name());
    }
}
