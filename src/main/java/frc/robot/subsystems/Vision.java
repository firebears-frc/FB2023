package frc.robot.subsystems;

import java.io.IOException;
import java.lang.annotation.Target;
import java.lang.reflect.Field;
import java.text.Format;
import java.util.Optional;

import org.ejml.dense.block.VectorOps_FDRB;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import frc.robot.util.VisionMap;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Vision extends SubsystemBase {
  PhotonCamera Camera;
  VisionMap VM;

  PhotonPoseEstimator poseEstimator;
  public Field2d Field = new Field2d();
  AprilTagFieldLayout layout;

  private boolean hasTarget = false;

  /** Creates a new Vision. */
  public Vision(String CamName) {
    Camera = new PhotonCamera(CamName);
    VM = new VisionMap();
    SmartDashboard.putData("VisionEyes",Field);
    
    
    try {
      layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
      poseEstimator = new PhotonPoseEstimator(
        layout, 
        PoseStrategy.CLOSEST_TO_LAST_POSE, 
        Camera, 
        new Transform3d(
          new Translation3d(
            Units.inchesToMeters(0),
            Units.feetToMeters(4),
            Units.feetToMeters(0)
          ),
          new Rotation3d()
        )
      );
      poseEstimator.setLastPose(new Pose3d());
      
    } catch (IOException e) {
      // TODO Auto-generated catch block
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

    SmartDashboard.putBoolean("PhotonVision Works", Camera.isConnected());
    
    PhotonPipelineResult result = Camera.getLatestResult();
    if(result.hasTargets()){
      hasTarget = true;
      //VM.onVisionTargetSeen(result.getBestTarget());
      Optional<EstimatedRobotPose> pose = poseEstimator.update();
      if(pose.isPresent()){
        //Pose is valid
        Pose3d p3d = pose.get().estimatedPose;
        //SmartDashboard.putString("VisionEayes",("" + p3d.toPose2d().getX() + ',' + p3d.toPose2d().getY()));
        Field.setRobotPose(p3d.toPose2d());
        poseEstimator.setLastPose(p3d.toPose2d());
      }
    } else {
      hasTarget = false;
    }
  }
}