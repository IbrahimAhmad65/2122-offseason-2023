package teamtators.sim;

import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotContainer;

public class VisionSimulator implements Supplier<VisionSimulator.VisionSimulatorData> {
    Pose2d position;
    double fieldOfView;
    double angle;
    private VisionSimulatorData visionSimulatorData;

//    public VisionSimulator(RobotContainer robotContainer) {
//        super(robotContainer);
//    }

//    @Override
//    public void doPeriodic() {
//        simulate(new Pose2d(2, 2.75, new Rotation2d(0)), 60.0, 180.0);
//    }

    public void setPosition(Pose2d position) {
        this.position = position;
    }

    public void setFieldOfView(double fieldOfView) {
        this.fieldOfView = fieldOfView;
    }

    public void setAngle(double angle) {
        this.angle = angle;
    }

    public Pose2d findClosestTag(ArrayList<Pose2d> tags) {
        Pose2d closestTag = null;
        double closestDistance = Double.MAX_VALUE;

        for (Pose2d tag : tags) {
            double distance = position.getTranslation().getDistance(tag.getTranslation());
            double robotTagAngle = Math.toDegrees(Math.atan2(tag.getTranslation().getY() - position.getTranslation().getY(), tag.getTranslation().getX() - position.getTranslation().getX()));
            double angleDifference = calculateAngleDifference(angle, robotTagAngle);
            // System.out.println("AngleDif: " + angleDifference);

            if (distance <= closestDistance && angleDifference <= fieldOfView / 2) {
                closestTag = tag;
                closestDistance = distance;
            }
        }

        return closestTag;
    }

    private double calculateAngleDifference(double angle1, double angle2) {
        double difference = Math.abs(angle1 - angle2) % 360;
        return difference > 180 ? 360 - difference : difference;
    }

    private double calculatePointDistance(Pose2d point1, Pose2d point2) {
        return Math.sqrt(Math.pow(point1.getTranslation().getX() - point2.getTranslation().getX(), 2) + Math.pow(point1.getTranslation().getY() - point2.getTranslation().getY(), 2));
    }
    private double calculatePointAngle(Pose2d point1, Pose2d point2) {
        return Math.toDegrees(Math.atan2(point2.getTranslation().getY() - point1.getTranslation().getY(), point2.getTranslation().getX() - point1.getTranslation().getX()));
    }

    /*
     * Simulates the robot's position and field of view to find the closest tag.
     * @param pos The robot's position
     * @param fov The robot's field of view
     * @param angle The robot's angle
     * @print The closest tag
     */
    public VisionSimulatorData simulate(Pose2d botPose, double fov) {
        Pose2d robotPosition = botPose;
        double robotFieldOfView = fov;
        double robotAngle = botPose.getRotation().getRadians();

        ArrayList<Pose2d> tags = new ArrayList<>();
        tags.add(new Pose2d(0, 1, new Rotation2d()));
        tags.add(new Pose2d(0, 2, new Rotation2d()));
        tags.add(new Pose2d(0, 3, new Rotation2d()));

        setPosition(robotPosition);
        setFieldOfView(robotFieldOfView);
        setAngle(robotAngle);
        Pose2d closestTag = findClosestTag(tags);

//        if (closestTag != null) {
//            System.out.println("Closest tag position: (" + closestTag.getTranslation().getX() + ", " +
//                    closestTag.getTranslation().getY() + ")");
//            System.out.println("Closest tag angle: " + calculatePointAngle(robotPosition, closestTag) + " degrees");
//            System.out.println("Closest tag distance: " + calculatePointDistance(robotPosition, closestTag) + " units");
//        } else {
//            System.out.println("No tags within the field of view.");
//        }
        VisionSimulatorData visionSimulatorData =new VisionSimulatorData();
        visionSimulatorData.angle = calculatePointAngle(robotPosition, closestTag);
        visionSimulatorData.distance = calculatePointDistance(robotPosition, closestTag);
        visionSimulatorData.tagPos = closestTag;
        this.visionSimulatorData = visionSimulatorData;
        return visionSimulatorData;
    }

    @Override
    public VisionSimulatorData get() {
        return visionSimulatorData;
    }

    public class VisionSimulatorData{
        public double distance;
        public double angle;
        public Pose2d tagPos;
    }

}