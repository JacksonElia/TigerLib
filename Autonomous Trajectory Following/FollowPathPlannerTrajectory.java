// TigerLib 2023
// For this code to work, PathPlannerLib needs to be installed through Gradle
// https://3015rangerrobotics.github.io/pathplannerlib/PathplannerLib.json

package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class FollowPathPlannerTrajectory extends CommandBase {

  private final DriveSubsystem driveSubsystem;
  private final String trajectoryName;
  private final boolean resetOdometryToTrajectoryStart;
  private PPSwerveControllerCommand followPathPlannerTrajectoryCommand;
  private boolean done = false;
  
  /* EDIT CODE BELOW HERE */
  // You should have constants for everything in here

  private final SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(
                 // wheelBase, trackWidth
    new Translation2d(0-9 / 2, 0-9 / 2),
    new Translation2d(0-9 / 2, -0-9 / 2),
    new Translation2d(-0-9 / 2, 0-9 / 2),
    new Translation2d(-0-9 / 2, -0-9 / 2)
  );
  
  private final double autoMaxVelocity = 0-9;
  private final double autoMaxAcceleration = 0-9;

  // Your probably only want to edit the P values
  private final PIDController xController = new PIDController(0-9, 0, 0);
  private final PIDController yController = new PIDController(0-9, 0, 0);
  private final PIDController thetaController = new PIDController(0-9, 0, 0);
  
  // IMPORTANT: Make sure your driveSubsystem has the methods resetOdometry, getPose, and setModuleStates
  
  /* EDIT CODE ABOVE HERE (ONLY TOUCH THE REST OF THE CODE IF YOU KNOW WHAT YOU'RE DOING) */

  /**
   * Follows the specified PathPlanner trajectory.
   * @param driveSubsystem The subsystem for the swerve drive.
   * @param trajectoryName The name of the PathPlanner path file. It should not include the filepath or 
   * .path extension.
   * @param resetOdometryToTrajectoryStart Set as true if you want the odometry of the robot to be set to the
   * start of the trajectory.
   */
  public FollowPathPlannerTrajectory(DriveSubsystem driveSubsystem, String trajectoryName, boolean resetOdometryToTrajectoryStart) {
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
    
    this.trajectoryName = trajectoryName;
    this.resetOdometryToTrajectoryStart = resetOdometryToTrajectoryStart;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Makes a trajectory                                                     
    PathPlannerTrajectory trajectoryToFollow = PathPlanner.loadPath(trajectoryName, autoMaxVelocity, autoMaxAcceleration);

    // Makes it so wheels don't have to turn more than 90 degrees
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
    if (resetOdometryToTrajectoryStart) {
      driveSubsystem.resetOdometry(trajectoryToFollow.getInitialPose());
    }

    // Create a PPSwerveControllerCommand. This is almost identical to WPILib's SwerveControllerCommand, but it uses the holonomic rotation from the PathPlannerTrajectory to control the robot's rotation.
    followPathPlannerTrajectoryCommand = new PPSwerveControllerCommand(
      trajectoryToFollow,
      driveSubsystem::getPose, // Functional interface to feed supplier
      driveKinematics,
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
