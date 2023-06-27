package teamtators.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSim extends SubsystemBase {
    
    public DriveSim(SwerveModulePosition pose) {

    }
    
    public Pose2d getDrivePose() {
        return new Pose2d(new Translation2d(0, null), new Rotation2d(0, 0));
    }

    
    /** Update our simulation. This should be run every robot loop in simulation. */
    @Override
  public void simulationPeriodic() {
    
  }
}
