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

  private DriveSubsystem driveSubsystem;
  private String trajectoryName;

  PPSwerveControllerCommand followTrajectoryPathPlannerCommand;
  private boolean done = false;

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

    // PID controllers
    PIDController xController = new PIDController(PathPlannerConstants.xControllerP, 0, 0);
    PIDController yController = new PIDController(PathPlannerConstants.yControllerP, 0, 0);
    PIDController thetaController = new PIDController(PathPlannerConstants.thetaControllerP, 0, 0);
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
