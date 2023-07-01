package frc.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.VisionConstants;

public class Vision extends SubsystemBase  {
    PhotonCamera camera;
    AprilTagFieldLayout aprilTagFieldLayout;
    Transform3d robotToCam;
    PhotonPoseEstimator photonPoseEstimator;

    public Vision() {
        //The name of the NetworkTable (for the string constructor) should be the same as the camera’s nickname (from the PhotonVision UI)
        camera = new PhotonCamera("photonvision");
        // The parameter for loadFromResource() will be different depending on the game.
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        robotToCam = new Transform3d(new Translation3d(VisionConstants.kRobotToCamX, VisionConstants.kRobotToCamY, VisionConstants.kRobotToCamZ), new Rotation3d(0,0,0)); 
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, robotToCam);
        photonPoseEstimator.setReferencePose(new Pose3d()); // IS THIS CORRECT?
    }

    //You should be updating your drivetrain pose estimator 
    //with the result from the RobotPoseEstimator every loop using addVisionMeasurement()
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        if (photonPoseEstimator == null) {
            // The field layout failed to load, so we cannot estimate poses.
            return Optional.empty();
        }

        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }
}