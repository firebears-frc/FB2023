package frc.robot.arm;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private static class ArmPosition {
        private final Rotation2d elbow;
        private final Rotation2d shoulder;

        public static ArmPosition fromDegrees(double elbow, double shoulder) {
            return new ArmPosition(Rotation2d.fromDegrees(elbow), Rotation2d.fromDegrees(shoulder));
        }

        public ArmPosition(Rotation2d elbow, Rotation2d shoulder) {
            this.elbow = elbow;
            this.shoulder = shoulder;
        }
    }

    private final static class Constants {
        public static final ArmPosition MANUAL_SPEED = ArmPosition.fromDegrees(1.0, 1.0); // per loop

        public static final ArmPosition SUBSTATION = ArmPosition.fromDegrees(65.0, 275.0);
        public static final ArmPosition STOW = ArmPosition.fromDegrees(20.0, 220.0);
        public static final ArmPosition GROUND_CONE = ArmPosition.fromDegrees(110.0, 230.0);
        public static final ArmPosition GROUND_CUBE = ArmPosition.fromDegrees(127.0, 224.0);
        public static final ArmPosition READY = ArmPosition.fromDegrees(122.0, 355.0);
        public static final ArmPosition HIGH = ArmPosition.fromDegrees(106.0, 319.0);
        public static final ArmPosition MID = ArmPosition.fromDegrees(76.0, 267.0);

        public static final double ANGLE_TOLERANCE = 2.5; // degrees

        public static final double UPPER_ARM_LENGTH = Units.inchesToMeters(30);
        public static final double FORE_ARM_LENGTH = Units.inchesToMeters(30);
    }

    private final Elbow elbow;
    private final Shoulder shoulder;
    @AutoLogOutput(key = "Arm/Mechanism")
    private final Mechanism2d arm;
    private final MechanismRoot2d armPivot;
    private final MechanismLigament2d upperArm, foreArm;

    public Arm() {
        elbow = new Elbow();
        shoulder = new Shoulder();

        arm = new Mechanism2d(0, 0);
        armPivot = arm.getRoot("Arm Pivot", 0, 0);
        upperArm = armPivot.append(
                new MechanismLigament2d("Upper Arm", Constants.UPPER_ARM_LENGTH, shoulder.getAngle().getDegrees()));
        foreArm = upperArm
                .append(new MechanismLigament2d("Fore Arm", Constants.FORE_ARM_LENGTH, elbow.getAngle().getDegrees()));
    }

    @AutoLogOutput(key = "Arm/OnTarget")
    private boolean onTarget() {
        return Math.abs(shoulder.getError().getDegrees()) < Constants.ANGLE_TOLERANCE
                && Math.abs(elbow.getError().getDegrees()) < Constants.ANGLE_TOLERANCE;
    }

    private void setPosition(ArmPosition position) {
        elbow.setAngle(position.elbow);
        shoulder.setAngle(position.shoulder);
    }

    @Override
    public void periodic() {
        elbow.periodic();
        shoulder.periodic();

        upperArm.setAngle(shoulder.getAngle());
        foreArm.setAngle(elbow.getAngle());
    }

    private Command positionCommand(ArmPosition position) {
        return run(() -> setPosition(position)).until(this::onTarget);
    }

    public Command groundCone() {
        return positionCommand(Constants.GROUND_CONE);
    }

    public Command groundCube() {
        return positionCommand(Constants.GROUND_CUBE);
    }

    public Command high() {
        return positionCommand(Constants.HIGH);
    }

    public Command mid() {
        return positionCommand(Constants.MID);
    }

    public Command ready() {
        return positionCommand(Constants.READY);
    }

    public Command stow() {
        return positionCommand(Constants.STOW);
    }

    public Command substation() {
        return positionCommand(Constants.SUBSTATION);
    }

    public Command defaultCommand(Supplier<Double> elbowChange, Supplier<Double> shoulderChange) {
        return run(() -> {
            Rotation2d elbowAngle = elbow.getTargetAngle();
            elbowAngle = elbowAngle.plus(Constants.MANUAL_SPEED.elbow.times(elbowChange.get()));

            Rotation2d shoulderAngle = shoulder.getTargetAngle();
            shoulderAngle = shoulderAngle.plus(Constants.MANUAL_SPEED.shoulder.times(shoulderChange.get()));

            setPosition(new ArmPosition(elbowAngle, shoulderAngle));
        });
    }
}
