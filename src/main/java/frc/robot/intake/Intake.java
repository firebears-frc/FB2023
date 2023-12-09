package frc.robot.intake;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.GamePiece;

public class Intake extends SubsystemBase {
    public static final class Constants {
        public static final int MOTOR_CAN_ID = 6;
    }

    protected enum IntakeState {
        INTAKE,
        EJECT,
        HOLD,
        STOP
    }

    // @AutoLogOutput(key = "Intake/State") TODO enum
    protected IntakeState state = IntakeState.STOP;
    // @AutoLogOutput(key = "Intake/ItemHeld") TODO enum
    protected GamePiece itemHeld = GamePiece.NONE;
    // @AutoLogOutput(key = "Intake/LastItemHeld") TODO enum
    protected GamePiece lastItemHeld = GamePiece.NONE;
    // @AutoLogOutput(key = "Intake/ItemWanted") TODO enum
    protected GamePiece itemWanted = GamePiece.NONE;

    // TODO enum
    @Override
    public void periodic() {
        Logger.recordOutput("Intake/State", state);
        Logger.recordOutput("Intake/ItemHeld", itemHeld);
        Logger.recordOutput("Intake/LastItemHeld", lastItemHeld);
        Logger.recordOutput("Intake/ItemWanted", itemWanted);
    }

    public Command intakeCone() {
        return runOnce(() -> {
            state = IntakeState.INTAKE;
            itemHeld = GamePiece.CONE;
            lastItemHeld = GamePiece.CONE;
            itemWanted = GamePiece.NONE;
        });
    }

    public Command intakeCube() {
        return runOnce(() -> {
            state = IntakeState.INTAKE;
            itemHeld = GamePiece.CUBE;
            lastItemHeld = GamePiece.CUBE;
            itemWanted = GamePiece.NONE;
        });
    }

    public Command hold() {
        return runOnce(() -> {
            state = IntakeState.HOLD;
        });
    }

    public Command eject() {
        return runOnce(() -> {
            state = IntakeState.EJECT;
            itemHeld = GamePiece.NONE;
        });
    }

    public Command stop() {
        return runOnce(() -> {
            state = IntakeState.STOP;
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
}
