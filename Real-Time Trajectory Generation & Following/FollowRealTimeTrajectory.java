// JackLib 2023

package frc.robot.commands.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import java.util.List;
import java.util.function.BooleanSupplier;

public class FollowRealTimeTrajectory extends CommandBase {

  // To get the x and y positions of where you want to go, you might have to use a subsystem
  private final DriveSubsystem driveSubsystem;
  private final BooleanSupplier whileHeldButtonBooleanSupplier;

  public FollowRealTimeTrajectory(DriveSubsystem driveSubsystem, BooleanSupplier whileHeldButtonBooleanSupplier) {
    this.driveSubsystem = driveSubsystem;
    this.whileHeldButtonBooleanSupplier = whileHeldButtonBooleanSupplier;
    addRequirements(this.driveSubsystem);
  }

  @Override
  public void initialize() {

    /* EDIT CODE BELOW HERE */

    // X and Y should be in meters
    // You might have to switch the x and y values and make them negative or positive
    // To get these 3 values, you should use the odometry or poseEstimator
    double startX = driveSubsystem.odometry.getPoseMeters().getY();
    double startY = driveSubsystem.odometry.getPoseMeters().getX();
    Rotation2d startRotation = driveSubsystem.odometry.getPoseMeters().getRotation();
    Pose2d start = new Pose2d(startY, startX, startRotation);

    // These values should be field relative, if they are robot relative add them to the start values
    double endX = driveSubsystem.odometry.getPoseMeters().getY();
    double endY = driveSubsystem.odometry.getPoseMeters().getX() + .5;
    Rotation2d endRotation = driveSubsystem.odometry.getPoseMeters().getRotation();
    Pose2d end = new Pose2d(endY, endX, endRotation);

    // If you want any middle waypoints in the trajectory, add them here
    List<Translation2d> middleWaypoints = List.of();

    // You should have constans or everything below here
    double driveMaxSpeedMetersPerSecond = 0-9;
    double driveMaxAccelerationMetersPerSecond = 0-9;
    double wheelBase = 0-9; // Distance between front and back wheels on robot
    double trackWidth = 0-9; // Distance between centers of right and left wheels on robot
    double turnMaxAngularSpeedRadiansPerSecond = 0-9;
    double turnMaxAngularSpeedRadiansPerSecondSquared = 0-9;
    
    // Your probably only want to edit the P values
    PIDController xController = new PIDController(0-9, 0, 0);
    PIDController yController = new PIDController(0-9, 0, 0);
    PIDController thetaController = new PIDController(0-9, 0, 0);

    // IMPORTANT: Make sure your driveSubsystem has the methods getPose and setModuleStates

    /* EDIT CODE ABOVE HERE (ONLY TOUCH THE REST OF THE CODE IF YOU KNOW WHAT YOU'RE DOING) */

    TrajectoryConfig config = new TrajectoryConfig(
      driveMaxSpeedMetersPerSecond,
      driveMaxAccelerationMetersPerSecond)
        // Add kinematics to ensure max speed is actually obeyed
        // .setKinematics(SwerveDriveConstants.kDriveKinematic)
        .setStartVelocity(0)
        .setEndVelocity(0);

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      start,
      middleWaypoints,
      end,
      config
    );

    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      new Translation2d(wheelBase / 2, trackWidth / 2),
      new Translation2d(wheelBase / 2, -trackWidth / 2),
      new Translation2d(-wheelBase / 2, trackWidth / 2),
      new Translation2d(-wheelBase / 2, -trackWidth / 2));

    final ProfiledPIDController thetaController = new ProfiledPIDController(
      thetaControllerP, thetaControllerI, thetaControllerD,
      new TrapezoidProfile.Constraints(turnMaxAngularSpeedRadiansPerSecond, turnMaxAngularSpeedRadiansPerSecondSquared)
    );
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    new RealTimeSwerveControllerCommand(
      trajectory,
      driveSubsystem::getPose, // Functional interface to feed supplier
      kinematics,
      // Position controllers
      new PIDController(xControllerP, xControllerI, xControllerD),
      new PIDController(yControllerP, yControllerI, yControllerD),
      thetaController,
      driveSubsystem::setModuleStates,
      whileHeldButtonBooleanSupplier,
      driveSubsystem
    ).schedule();

  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }

}
