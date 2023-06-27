package teamtators.sim;

import java.io.IOException;
import java.util.ArrayList;
import java.util.function.Supplier;

import org.photonvision.SimVisionSystem;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.VisionConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class VisionSimulator extends SubsystemBase {

        // Simulated Vision System.
    // Configure these to match your PhotonVision Camera,
    // pipeline, and LED setup.
    double camDiagFOV = 170.0; // degrees - assume wide-angle camera
    double camPitch = VisionConstants.kCameraPitchRadians; // degrees
    double camHeightOffGround = VisionConstants.kCameraHeightMeters; // meters
    double maxLEDRange = 20; // meters
    int camResolutionWidth = 640; // pixels
    int camResolutionHeight = 480; // pixels
    double minTargetArea = 10; // square pixels

    double targetWidth = Units.inchesToMeters(41.30) - Units.inchesToMeters(6.70); // meters
    double targetHeight = Units.inchesToMeters(98.19) - Units.inchesToMeters(81.19); // meters
    double tgtXPos = Units.feetToMeters(54);
    double tgtYPos = Units.feetToMeters(27 / 2) - Units.inchesToMeters(43.75) - Units.inchesToMeters(48.0 / 2.0);
    Pose3d farTargetPose =
            new Pose3d(
                    new Translation3d(tgtXPos, tgtYPos, VisionConstants.kTargetHeightMeters),
                    new Rotation3d(0.0, 0.0, 0.0));

    SimVisionSystem simVision; 


    public VisionSimulator(RobotContainer robotContainer) {
        simVision = new SimVisionSystem(
                    "photonvision",
                    camDiagFOV,
                    new Transform3d(
                            new Translation3d(0, 0, camHeightOffGround), new Rotation3d(0, camPitch, 0)),
                    maxLEDRange,
                    camResolutionWidth,
                    camResolutionHeight,
                    minTargetArea);

        try {
            simVision.addVisionTargets(AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile));
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }

    @Override
    public void simulationPeriodic() {
        simVision.processFrame(drivetrainSimulator.getPose());
    }


}