package frc.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import org.opencv.core.Mat;
import org.photonvision.EstimatedRobotPose;

import teamtators.sim.VisionSimulator;
import teamtators.util.Timer;

import java.util.Optional;
import java.util.function.Supplier;

public class PoseStuff implements Supplier<Pose2d>, Runnable{
    private  SwerveDrivePoseEstimator swerveDrivePoseEstimator;
    public  boolean sim = SimWriter.sim;
    private SwerveDrive swerveDrive;
    private Pose2d pose2d;
    private Vision phoVision;
    private boolean vision = false;

    public PoseStuff(SwerveDrive swerveDrive){
        this.swerveDrive = swerveDrive;
//        swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(swerveDrive.getSwerveDriveKinematics(),
//                swerveDrive.getSwerveDriveRotation(), new SwerveModulePosition[]{new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()},new Pose2d()
////                ,VecBuilder.fill(0.9, 0.9, 0.9), VecBuilder.fill(0.001, 0.001, 0.001)
//                );
//
//        if(sim){
//            visionSimulator = new VisionSimulator();
//        } else {
//            // vision subsystem
//        }
        pose2d = new Pose2d();
    }

    public void tick(){
        if(swerveDrive == null){
            return;
        } else if(swerveDrivePoseEstimator == null){
            swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(swerveDrive.getSwerveDriveKinematics(),
                    swerveDrive.getSwerveDriveRotation(), new SwerveModulePosition[]{new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()},new Pose2d()
//                ,VecBuilder.fill(0.9, 0.9, 0.9), VecBuilder.fill(0.001, 0.001, 0.001)
            );
        }
          pose2d = swerveDrivePoseEstimator.update(swerveDrive.getSwerveDriveRotation(),swerveDrive.getSwerveModulePositions());

//        System.out.println(swerveDrive.getSwerveModulePositions()[0].distanceMeters);

        if(vision && Math.random() > .95){
            System.out.println("here");
//            VisionSimulator.VisionSimulatorData data = visionSimulator.get();
//            Translation2d deltaPos = new Translation2d();
//            deltaPos.plus(new Translation2d(data.distance, new Rotation2d(data.angle))).div(-1).plus(data.tagPos.getTranslation());
            
        Optional<EstimatedRobotPose> result =
        phoVision.getEstimatedGlobalPose(swerveDrivePoseEstimator.getEstimatedPosition());

        if (result.isPresent()) {
            swerveDrivePoseEstimator.addVisionMeasurement(result.get().estimatedPose.toPose2d(), result.get().timestampSeconds);
        }
        Pose2d pose = swerveDrivePoseEstimator.getEstimatedPosition();
        System.out.println("(" + pose.getX() + "," + pose.getY()+")");

        }
    }
    public Pose2d getPose2d(){
        return swerveDrivePoseEstimator.getEstimatedPosition();
    }
    @Override
    public void run(){
        while (Thread.currentThread().isAlive()){
            tick();
//            System.out.println("herew");
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }

    public void setSwerveDrive(SwerveDrive swerveDrive){
        this.swerveDrive = swerveDrive;
    }
    @Override
    public Pose2d get() {
        return swerveDrivePoseEstimator.getEstimatedPosition();
    }
}
