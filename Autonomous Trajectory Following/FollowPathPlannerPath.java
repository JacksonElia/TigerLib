// This code can be uncommented and used once a DriveSubsystem has been created.

// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.subsystems.DriveSubsystem;

public class FollowPathPlannerPath extends CommandBase {

  private DriveSubsystem driveSubsystem;
  private String filePath;
  private boolean zeroInitialPose;

  PPSwerveControllerCommand followTrajectoryPathPlannerCommand;
  private boolean done = false;

  /** Creates a new FollowTrajectoryPathPlanner. */
  public FollowPathPlannerPath(DriveSubsystem driveSubsystem, String filePath, boolean zeroInitialPose) {
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
    
    this.filePath = filePath;
    this.zeroInitialPose = zeroInitialPose;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Makes a trajectory                                                     
    PathPlannerTrajectory trajectoryToFollow = PathPlanner.loadPath(filePath, PathPlannerConstants.autoMaxVelocity, PathPlannerConstants.autoMaxAcceleration);

    // Resets the pose of the robot if true
    if (zeroInitialPose) {
      driveSubsystem.resetOdometry(trajectoryToFollow.getInitialPose());
    }

    // PID controllers
    PIDController xController = new PIDController(PathPlannerConstants.xControllerP, 0, 0);
    PIDController yController = new PIDController(PathPlannerConstants.yControllerP, 0, 0);
    /*Profiled*/PIDController thetaController = new PIDController(
    PathPlannerConstants.thetaControllerP, 0, 0);//, PathPlannerConstants.thetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI); // Makes it so wheels don't have to turn more than 90 degrees

    // Create a PPSwerveControllerCommand. This is almost identical to WPILib's SwerveControllerCommand, but it uses the holonomic rotation from the PathPlannerTrajectory to control the robot's rotation.
    followTrajectoryPathPlannerCommand = new PPSwerveControllerCommand(
      trajectoryToFollow,
      driveSubsystem::getPose, // Functional interface to feed supplier
      DriveConstants.driveKinematics,
      xController,
      yController,
      thetaController,
      driveSubsystem::setModuleStates,
      false,
      driveSubsystem
    );
    
    followTrajectoryPathPlannerCommand.schedule();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    done = followTrajectoryPathPlannerCommand.isFinished();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
