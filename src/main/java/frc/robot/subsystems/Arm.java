package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private static class Constants {
        public static final double ELBOW_MANUAL_SPEED = 1.0; // degrees per loop
        public static final double SHOULDER_MANUAL_SPEED = 1.0; // degrees per loop

        public static final double SHOULDER_SUBSTATION = 65.0; // degrees
        public static final double ELBOW_SUBSTATION = 275.0; // degrees
        public static final double SHOULDER_STOW = 20.0; // degrees
        public static final double ELBOW_STOW = 220.0; // degrees
        public static final double SHOULDER_GROUND_CONE = 110.0; // degrees
        public static final double ELBOW_GROUND_CONE = 230.0; // degrees
        public static final double SHOULDER_GROUND_CUBE = 127.0; // degrees
        public static final double ELBOW_GROUND_CUBE = 224.0; // degrees
        public static final double SHOULDER_READY = 122.0; // degrees
        public static final double ELBOW_READY = 355.0; // degrees
        public static final double SHOULDER_HIGH = 106.0; // degrees
        public static final double ELBOW_HIGH = 319.0; // degrees
        public static final double SHOULDER_MID = 76.0; // degrees
        public static final double ELBOW_MID = 267.0; // degrees

        public static final double ANGLE_TOLERANCE = 2.5; // degrees
    }

    private final ArmElbow elbow;
    private final ArmShoulder shoulder;

    public Arm() {
        elbow = new ArmElbow();
        shoulder = new ArmShoulder();
    }

    private boolean onTarget() {
        return Math.abs(shoulder.getError()) < Constants.ANGLE_TOLERANCE
                && Math.abs(elbow.getError()) < Constants.ANGLE_TOLERANCE;
    }

    private void setAngles(double elbowAngle, double shoulderAngle) {
        elbow.setAngle(elbowAngle);
        shoulder.setAngle(shoulderAngle);
    }

    @Override
    public void periodic() {
        elbow.periodic();
        shoulder.periodic();
    }

    private class PositionCommand extends CommandBase {
        private final Arm arm;
        private final double elbowSetpoint;
        private final double shoulderSetpoint;

        public PositionCommand(Arm arm, double elbowSetpoint, double shoulderSetpoint) {
            this.arm = arm;
            this.elbowSetpoint = elbowSetpoint;
            this.shoulderSetpoint = shoulderSetpoint;
            addRequirements(arm);
        }

        @Override
        public void execute() {
            arm.setAngles(elbowSetpoint, shoulderSetpoint);
        }

        @Override
        public boolean isFinished() {
            return arm.onTarget();
        }
    }

    public Command groundCone() {
        return new PositionCommand(
                this,
                Constants.ELBOW_GROUND_CONE,
                Constants.SHOULDER_GROUND_CONE);
    }

    public Command groundCube() {
        return new PositionCommand(
                this,
                Constants.ELBOW_GROUND_CUBE,
                Constants.SHOULDER_GROUND_CUBE);
    }

    public Command high() {
        return new PositionCommand(
                this,
                Constants.ELBOW_HIGH,
                Constants.SHOULDER_HIGH);
    }

    public Command mid() {
        return new PositionCommand(
                this,
                Constants.ELBOW_MID,
                Constants.SHOULDER_MID);
    }

    public Command ready() {
        return new PositionCommand(
                this,
                Constants.ELBOW_READY,
                Constants.SHOULDER_READY);
    }

    public Command stow() {
        return new PositionCommand(
                this,
                Constants.ELBOW_STOW,
                Constants.SHOULDER_STOW);
    }

    public Command substation() {
        return new PositionCommand(
                this,
                Constants.ELBOW_SUBSTATION,
                Constants.SHOULDER_SUBSTATION);
    }

    public Command defaultCommand(Supplier<Double> elbowChange, Supplier<Double> shoulderChange) {
        return new RunCommand(() -> {
            double elbowAngle = elbow.getTargetAngle();
            elbowAngle += elbowChange.get() * Constants.ELBOW_MANUAL_SPEED;

            double shoulderAngle = shoulder.getTargetAngle();
            shoulderAngle += shoulderChange.get() * Constants.SHOULDER_MANUAL_SPEED;

            setAngles(elbowAngle, shoulderAngle);
        }, this);
    }
}
