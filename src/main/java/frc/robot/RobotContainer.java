// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.commands.ArmGoToPosition;
import frc.commands.BaseDrive;
import frc.commands.WristToAngle;
import frc.constants.OperatorInterfaceConstants;
import frc.subsystems.*;
import teamtators.util.JoystickModifiers;

public class RobotContainer {
  private final Arm arm;
  private final SimWriter simWriter;
  private final Wrist wrist;
  private final Claw claw ;

  public final BaseDrive teleDrive;
  private SwerveDrive swerveDrive;
private OperatorInterface operatorInterface;
  public SequentialCommandGroup armDoThings;
  public SequentialCommandGroup pickFromChamber;
  public RobotContainer() {

    swerveDrive = new SwerveDrive();
    operatorInterface = new OperatorInterface(swerveDrive.getGyro());
    claw = new Claw();
    arm = new Arm();
    wrist = new Wrist();
    simWriter = new SimWriter(arm,wrist,claw);
    teleDrive = new BaseDrive(swerveDrive,() -> {
      return ChassisSpeeds.fromFieldRelativeSpeeds(1,1,0,new Rotation2d(0));
    });

    pickFromChamber = new SequentialCommandGroup(
            new ArmGoToPosition(arm,-Math.PI/2-Math.PI/6,.8),
            new WristToAngle(wrist,Math.PI),
            new ArmGoToPosition(arm,-Math.PI/2-Math.PI/6,.9),
            new ArmGoToPosition(arm,-Math.PI/2-Math.PI/6,.8),
            new ArmGoToPosition(arm,-Math.PI/2,.8)
            );
    armDoThings = new SequentialCommandGroup(
            pickFromChamber
    );
    configureBindings();

  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

    // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.


  public Command getAutoFromName() {
    
// Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
    driveSubsystem::getPose, // Pose2d supplier
    driveSubsystem::resetPose, // Pose2d consumer, used to reset odometry at the beginning of auto
    driveSubsystem.kinematics, // SwerveDriveKinematics
    new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
    new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
    driveSubsystem::setModuleStates, // Module states consumer used to output to the drive subsystem
    eventMap,
    true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
    driveSubsystem // The drive subsystem. Used to properly set the requirements of path following commands
);

  }
}
