// JackLib 2023
// For this code to work, PathPlannerLib needs to be installed through Gradle
// https://3015rangerrobotics.github.io/pathplannerlib/PathplannerLib.json

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.subsystems.drive.DriveSubsystemImpl;

public class FollowRealTimeTrajectory extends CommandBase {

  private final DriveSubsystemImpl driveSubsystem;
  private final boolean resetOdometryToTrajectoryStart;
  private final BooleanSupplier isFinished;
  
  /**
   * Follows the specified PathPlanner trajectory.
   * @param driveSubsystem The subsystem for the swerve drive.
   * @param trajectoryName The name of the PathPlanner path file. It should not include the filepath or 
   * .path extension.
   * @param resetOdometryToTrajectoryStart Set as true if you want the odometry of the robot to be set to the
   * start of the trajectory.
   */
  public FollowRealTimeTrajectory(DriveSubsystemImpl driveSubsystem, boolean resetOdometryToTrajectoryStart, BooleanSupplier isFinished) {
    this.driveSubsystem = driveSubsystem;
    this.isFinished = isFinished;
    this.resetOdometryToTrajectoryStart = resetOdometryToTrajectoryStart;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    /* EDIT CODE BELOW HERE */

    // X and Y should be in meters
    // To get these 3 values, you should use the odometry or poseEstimator
    double startX = driveSubsystem.getPose().getX();
    double startY = driveSubsystem.getPose().getY();
    Rotation2d startRotation = driveSubsystem.getPose().getRotation();
    Translation2d start = new Translation2d(startX, startY);
    // These values should be field relative, if they are robot relative add them to the start values
    double endX = driveSubsystem.getPose().getX() + 2;
    double endY = driveSubsystem.getPose().getY();
    Rotation2d endRotation = Rotation2d.fromDegrees(45);
    Translation2d end = new Translation2d(endX, endY);
    
    // Your probably only want to edit the P values
    PIDController xController = new PIDController(TrajectoryConstants.xControllerP, 0, 0);
    PIDController yController = new PIDController(TrajectoryConstants.yControllerP, 0, 0);
    PIDController thetaController = new PIDController(TrajectoryConstants.thetaControllerP, 0, 0);

    SwerveDriveKinematics kinematics = DriveConstants.driveKinematics;
    
    // This should be fine, but is here just in case so the robot doesn't crash during a match
    try {
      // Makes a trajectory that factors in holonomic rotation
      PathPlannerTrajectory trajectoryToFollow = PathPlanner.generatePath(
        new PathConstraints(TrajectoryConstants.autoMaxVelocity, TrajectoryConstants.autoMaxAcceleration),
        // position, heading (direction of travel)
        new PathPoint(start, startRotation, Rotation2d.fromDegrees(0)),
        // If you want any middle waypoints, add more pathpoints here
        new PathPoint(end, endRotation, Rotation2d.fromDegrees(0))
      );                                               

    // IMPORTANT: Make sure your driveSubsystem has the methods getPose and setModuleStates

    /* EDIT CODE ABOVE HERE (ONLY TOUCH THE REST OF THE CODE IF YOU KNOW WHAT YOU'RE DOING) */

      // Makes it so wheels don't have to turn more than 90 degrees
      thetaController.enableContinuousInput(-Math.PI, Math.PI);

      new RealTimePPSwerveControllerCommand(
        trajectoryToFollow,
        driveSubsystem::getPose, // Functional interface to feed supplier
        kinematics,
        xController,
        yController,
        thetaController,
        driveSubsystem::setModuleStates,
        false,
        isFinished,
        new Pose2d(),
        driveSubsystem
      ).schedule();
    } catch(Exception e) {
      SmartDashboard.putString("Trajectory Error Message", e.getLocalizedMessage());
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
