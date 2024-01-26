package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
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
    PhotonCamera Camera;
    PhotonPoseEstimator poseEstimator;
    AprilTagFieldLayout layout;

    private boolean hasTarget = false;

    private final DriveSubsystem m_chassis;

    /** Creates a new Vision. */
    public Vision(String CamName, DriveSubsystem chassis) {
        Camera = new PhotonCamera(CamName);
        m_chassis = chassis;

        try {
            layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
            poseEstimator = new PhotonPoseEstimator(
                    layout,
                    PoseStrategy.MULTI_TAG_PNP_ON_RIO,
                    Camera,
                    new Transform3d(
                            new Translation3d(
                                    Units.inchesToMeters(0),
                                    Units.feetToMeters(0),
                                    Units.feetToMeters(0)),
                            new Rotation3d()));
            poseEstimator.setLastPose(new Pose3d());

        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public boolean hasTarget() {
        return hasTarget;
    }

    void setLastPose(Pose2d p2d) {
        poseEstimator.setLastPose(p2d);
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(!Camera.isConnected()) return;
    
    PhotonPipelineResult result = Camera.getLatestResult();
    if(result.hasTargets()){
      hasTarget = true;
      Optional<EstimatedRobotPose> pose = poseEstimator.update();
      if(pose.isPresent()){
        //Pose is valid
        Pose3d p3d = pose.get().estimatedPose;
        poseEstimator.setLastPose(p3d.toPose2d());
        m_chassis.resetOdometry(p3d.toPose2d());
      }
    } else {
      hasTarget = false;
    }
  }
}
