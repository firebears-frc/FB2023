package frc.robot.subsystems;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
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
        public static final Rotation2d ELBOW_MANUAL_SPEED = Rotation2d.fromDegrees(1.0); // per loop
        public static final Rotation2d SHOULDER_MANUAL_SPEED = Rotation2d.fromDegrees(1.0); // per loop

        public static final Rotation2d SHOULDER_SUBSTATION = Rotation2d.fromDegrees(65.0);
        public static final Rotation2d ELBOW_SUBSTATION = Rotation2d.fromDegrees(275.0);
        public static final Rotation2d SHOULDER_STOW = Rotation2d.fromDegrees(20.0);
        public static final Rotation2d ELBOW_STOW = Rotation2d.fromDegrees(220.0);
        public static final Rotation2d SHOULDER_GROUND_CONE = Rotation2d.fromDegrees(110.0);
        public static final Rotation2d ELBOW_GROUND_CONE = Rotation2d.fromDegrees(230.0);
        public static final Rotation2d SHOULDER_GROUND_CUBE = Rotation2d.fromDegrees(127.0);
        public static final Rotation2d ELBOW_GROUND_CUBE = Rotation2d.fromDegrees(224.0);
        public static final Rotation2d SHOULDER_READY = Rotation2d.fromDegrees(122.0);
        public static final Rotation2d ELBOW_READY = Rotation2d.fromDegrees(355.0);
        public static final Rotation2d SHOULDER_HIGH = Rotation2d.fromDegrees(106.0);
        public static final Rotation2d ELBOW_HIGH = Rotation2d.fromDegrees(319.0);
        public static final Rotation2d SHOULDER_MID = Rotation2d.fromDegrees(76.0);
        public static final Rotation2d ELBOW_MID = Rotation2d.fromDegrees(267.0);

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
        upperArm = armPivot.append(
                new MechanismLigament2d("Upper Arm", Constants.UPPER_ARM_LENGTH, shoulder.getAngle().getDegrees()));
        foreArm = upperArm
                .append(new MechanismLigament2d("Fore Arm", Constants.FORE_ARM_LENGTH, elbow.getAngle().getDegrees()));
    }

    private boolean onTarget() {
        return Math.abs(shoulder.getError().getDegrees()) < Constants.ANGLE_TOLERANCE
                && Math.abs(elbow.getError().getDegrees()) < Constants.ANGLE_TOLERANCE;
    }

    private void setAngles(Rotation2d elbowAngle, Rotation2d shoulderAngle) {
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

    private Command positionCommand(Rotation2d elbowSetpoint, Rotation2d shoulderSetpoint) {
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
            Rotation2d elbowAngle = elbow.getTargetAngle();
            elbowAngle = elbowAngle.plus(Constants.ELBOW_MANUAL_SPEED.times(elbowChange.get()));

            Rotation2d shoulderAngle = shoulder.getTargetAngle();
            shoulderAngle = shoulderAngle.plus(Constants.SHOULDER_MANUAL_SPEED.times(shoulderChange.get()));

            setAngles(elbowAngle, shoulderAngle);
        }, this);
    }
}
