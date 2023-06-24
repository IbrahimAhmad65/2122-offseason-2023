package frc.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import teamtators.sim.VisionSimulator;
import teamtators.util.Timer;

import java.util.function.Supplier;

public class PoseStuff implements Supplier<Pose2d>{
    private final SwerveDrivePoseEstimator swerveDrivePoseEstimator;
    public  boolean sim = SimWriter.sim;
    private SwerveDrive swerveDrive;
    private Pose2d pose2d;
    private boolean vision = false;
    private Timer timer;

    public PoseStuff(SwerveDrive swerveDrive){
        timer = new Timer();
        swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(swerveDrive.getSwerveDriveKinematics(),
                swerveDrive.getSwerveDriveRotation(), new SwerveModulePosition[]{new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()},new Pose2d());
//
//        if(sim){
//            visionSimulator = new VisionSimulator();
//        } else {
//            // vision subsystem
//        }
        timer.start();

    }

    public void tick(){
       pose2d = swerveDrivePoseEstimator.update(swerveDrive.getSwerveDriveRotation(),swerveDrive.getSwerveModulePositions());
        if(vision){
//            VisionSimulator.VisionSimulatorData data = visionSimulator.get();
//            Translation2d deltaPos = new Translation2d();
//            deltaPos.plus(new Translation2d(data.distance, new Rotation2d(data.angle))).div(-1).plus(data.tagPos.getTranslation());
            Pose2d fromVision = new Pose2d(new Translation2d(5,5),new Rotation2d(5));
            swerveDrivePoseEstimator.addVisionMeasurement(fromVision, timer.get());
        }
    }
    public Pose2d getPose2d(){
        return pose2d;
    }


    @Override
    public Pose2d get() {
        return pose2d;
    }
}
