package frc.robot.subsystems;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
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

        public static final double UPPER_ARM_LENGTH = Units.inchesToMeters(30);
        public static final double FORE_ARM_LENGTH = Units.inchesToMeters(30);
    }

    private final ArmElbow elbow;
    private final ArmShoulder shoulder;
    private final Mechanism2d arm;
    private final MechanismRoot2d armPivot;
    private final MechanismLigament2d upperArm, foreArm;

    public Arm() {
        elbow = new ArmElbow();
        shoulder = new ArmShoulder();

        arm = new Mechanism2d(0, 0);
        armPivot = arm.getRoot("Arm Pivot", 0, 0);
        upperArm = armPivot.append(new MechanismLigament2d("Upper Arm", Constants.UPPER_ARM_LENGTH, shoulder.getAngle()));
        foreArm = upperArm.append(new MechanismLigament2d("Fore Arm", Constants.FORE_ARM_LENGTH, elbow.getAngle()));
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

        upperArm.setAngle(shoulder.getAngle());
        foreArm.setAngle(elbow.getAngle());
        Logger.getInstance().recordOutput("Arm/Mechanism", arm);
    }

    private Command positionCommand(double elbowSetpoint, double shoulderSetpoint) {
        return new FunctionalCommand(null, () -> setAngles(elbowSetpoint, shoulderSetpoint), null, this::onTarget,
                this);
    }

    public Command groundCone() {
        return positionCommand(
                Constants.ELBOW_GROUND_CONE,
                Constants.SHOULDER_GROUND_CONE);
    }

    public Command groundCube() {
        return positionCommand(
                Constants.ELBOW_GROUND_CUBE,
                Constants.SHOULDER_GROUND_CUBE);
    }

    public Command high() {
        return positionCommand(
                Constants.ELBOW_HIGH,
                Constants.SHOULDER_HIGH);
    }

    public Command mid() {
        return positionCommand(
                Constants.ELBOW_MID,
                Constants.SHOULDER_MID);
    }

    public Command ready() {
        return positionCommand(
                Constants.ELBOW_READY,
                Constants.SHOULDER_READY);
    }

    public Command stow() {
        return positionCommand(
                Constants.ELBOW_STOW,
                Constants.SHOULDER_STOW);
    }

    public Command substation() {
        return positionCommand(
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
