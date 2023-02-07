// For this code to work, PathPlannerLib needs to be installed through Gradle
// https://3015rangerrobotics.github.io/pathplannerlib/PathplannerLib.json

package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.subsystems.DriveSubsystem;

public class FollowPathPlannerTrajectory extends CommandBase {

  private final DriveSubsystem driveSubsystem;
  private final PPSwerveControllerCommand followPathPlannerTrajectoryCommand;
  private final String trajectoryName;
  private final boolean done = false;
  
  // EDIT CODE BELOW HERE
  
  // Your probably on want to edit the P values
  private final PIDController xController = new PIDController(0-9, 0, 0);
  private final PIDController yController = new PIDController(0-9, 0, 0);
  private final PIDController thetaController = new PIDController(0-9, 0, 0);
  
  // EDIT CODE ABOVE HERE

  /**
   * Follows the specified PathPlanner trajectory.
   * @param driveSubsystem The subsystem for the swerve drive.
   * @param trajectoryName The name of the PathPlanner path file. It should not include the filepath or .path extension.
   */
  public FollowPathPlannerTrajectory(DriveSubsystem driveSubsystem, String trajectoryName) {
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
    
    this.trajectoryName = trajectoryName;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Makes a trajectory                                                     
    PathPlannerTrajectory trajectoryToFollow = PathPlanner.loadPath(trajectoryName, PathPlannerConstants.autoMaxVelocity, PathPlannerConstants.autoMaxAcceleration);

    // Makes it so wheels don't have to turn more than 90 degrees
    thetaController.enableContinuousInput(-Math.PI, Math.PI); 

    // Create a PPSwerveControllerCommand. This is almost identical to WPILib's SwerveControllerCommand, but it uses the holonomic rotation from the PathPlannerTrajectory to control the robot's rotation.
    followPathPlannerTrajectoryCommand = new PPSwerveControllerCommand(
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
    
    followPathPlannerTrajectoryCommand.schedule();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    done = followPathPlannerTrajectoryCommand.isFinished();
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
