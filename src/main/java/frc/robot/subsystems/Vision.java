package frc.robot.subsystems;

import java.io.IOException;
import java.util.List;
import java.util.Optional;
import java.util.function.BiConsumer;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private static class Constants {
        public static final String NAME = "MainC";
    }

    PhotonCamera camera;
    PhotonPoseEstimator poseEstimator;
    EstimatedRobotPose lastResult;
    BiConsumer<Pose2d, Double> consumer;

    public Vision(BiConsumer<Pose2d, Double> consumer) {
        this.consumer = consumer;
        lastResult = new EstimatedRobotPose(new Pose3d(), 0, List.of());

        camera = new PhotonCamera(Constants.NAME);

        AprilTagFieldLayout fieldLayout = null;
        try {
            fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (IOException e) {
            System.out.println("Failed to load AprilTag layout!");
            e.printStackTrace();
            return;
        }

        poseEstimator = new PhotonPoseEstimator(
                fieldLayout,
                PoseStrategy.MULTI_TAG_PNP,
                camera,
                new Transform3d(new Translation3d(
                        Units.inchesToMeters(0),
                        Units.inchesToMeters(0),
                        Units.inchesToMeters(0)),
                        new Rotation3d()));
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    private enum Status {
        NOT_CONNECTED,
        NO_TARGETS,
        NO_POSE_RESULT,
        NO_CONSUMER,
        POSE_FOUND
    }

    private Status updatePose() {
        boolean connected = camera.isConnected();
        if (connected)
            return Status.NOT_CONNECTED;

        PhotonPipelineResult result = camera.getLatestResult();
        if (!result.hasTargets())
            return Status.NOT_CONNECTED;

        Optional<EstimatedRobotPose> poseResult = poseEstimator.update();
        if (!poseResult.isPresent())
            return Status.NO_POSE_RESULT;

        lastResult = poseResult.get();
        if (consumer == null)
            return Status.NO_CONSUMER;

        consumer.accept(lastResult.estimatedPose.toPose2d(), lastResult.timestampSeconds);
        return Status.POSE_FOUND;
    }

    @Override
    public void periodic() {
        Status status = updatePose();

        Logger logger = Logger.getInstance();
        logger.recordOutput("Vision/Status", status.name());
        logger.recordOutput("Vision/Pose", lastResult.estimatedPose);
        logger.recordOutput("Vision/Timestamp", lastResult.timestampSeconds);
        logger.recordOutput("Vision/TargetsFound", lastResult.targetsUsed.size());

        // Map the target objects to their integer ID, sort them, then store in a list
        List<Integer> visibleTags = lastResult.targetsUsed.stream().map(target -> target.getFiducialId()).sorted()
                .toList();
        // Map the IDs to a string, then join with commas
        logger.recordOutput("Vision/VisibleIDs",
                String.join(", ", visibleTags.stream().map(id -> id.toString()).toList()));

        // Set only the IDs for the visible tags
        boolean[] tags = new boolean[poseEstimator.getFieldTags().getTags().size()];
        visibleTags.stream().forEach(id -> tags[id] = true);
        logger.recordOutput("Vision/VisibleTargets", tags);
    }
}
