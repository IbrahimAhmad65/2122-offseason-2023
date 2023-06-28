package teamtators.sim;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.VisionConstants;
import frc.robot.RobotContainer;
import frc.subsystems.SwerveDrive;
import frc.subsystems.Vision;
import java.io.IOException;
import org.photonvision.SimVisionSystem;

 
 /** Represents a differential drive style drivetrain. */
 public class DriveSim extends SubsystemBase {
 
       // Simulated Vision System.
    // Configure these to match your PhotonVision Camera,
    // pipeline, and LED setup.
    private double camDiagFOV = 170.0; // degrees - assume wide-angle camera
    private double camPitch = VisionConstants.kCameraPitchRadians; // degrees
    private double camHeightOffGround = VisionConstants.kCameraHeightMeters; // meters
    private double maxLEDRange = 20; // meters
    private int camResolutionWidth = 640; // pixels
    private int camResolutionHeight = 480; // pixels
    private double minTargetArea = 10; // square pixels

    private double targetWidth = Units.inchesToMeters(41.30) - Units.inchesToMeters(6.70); // meters
    private double targetHeight = Units.inchesToMeters(98.19) - Units.inchesToMeters(81.19); // meters
    private double tgtXPos = Units.feetToMeters(54);
    private double tgtYPos = Units.feetToMeters(27 / 2) - Units.inchesToMeters(43.75) - Units.inchesToMeters(48.0 / 2.0);
    private Pose3d farTargetPose =
            new Pose3d(
                    new Translation3d(tgtXPos, tgtYPos, VisionConstants.kTargetHeightMeters),
                    new Rotation3d(0.0, 0.0, 0.0));

    private Vision phoVision;
    private SwerveDrive swerveDrive;
 
     SimVisionSystem simVision;

 
     /**
      * Constructs a differential drive object. Sets the encoder distance per pulse and resets the
      * gyro.
      */
     public DriveSim(RobotContainer robotContainer) {
        super();
                
        swerveDrive = robotContainer.getSwerveDrive();
        phoVision = robotContainer.getVision();
 
         // Only simulate our PhotonCamera in simulation
         if (RobotBase.isSimulation()) {
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
     }
 
     /** Update our simulation. This should be run every robot loop in simulation. */
     public void simulationPeriodic() {
        swerveDrive.getPoseStuff().tick();
        simVision.processFrame(swerveDrive.getPoseStuff().getPose2d());
     }

 }