// JackLib 2023 
// This command is almost identical to the Pathplanner PPSwerveControllerCommand 
// The key difference is that this command can be canceled mid-trajectory 
// Do not touch this unless you know what you're doing.

package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.drive.DriveSubsystem;

import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class RealTimePPSwerveControllerCommand extends CommandBase {
  private final Timer timer = new Timer();
  private final PathPlannerTrajectory trajectory;
  private final Supplier<Pose2d> poseSupplier;
  private final SwerveDriveKinematics kinematics;
  private final PPHolonomicDriveController controller;
  private final Consumer<SwerveModuleState[]> outputModuleStates;
  private final Consumer<ChassisSpeeds> outputChassisSpeeds;
  private final boolean useKinematics;
  private final boolean useAllianceColor;
  private final BooleanSupplier isFinished;
  private final Pose2d endPose;
  private final DriveSubsystem driveSubsystem;

  private PathPlannerTrajectory transformedTrajectory;

  private static Consumer<PathPlannerTrajectory> logActiveTrajectory = null;
  private static Consumer<Pose2d> logTargetPose = null;
  private static Consumer<ChassisSpeeds> logSetpoint = null;
  private static BiConsumer<Translation2d, Rotation2d> logError =
      RealTimePPSwerveControllerCommand::defaultLogError;

  /**
   * Constructs a new RealTimePPSwerveControllerCommand that when executed will follow the provided
   * trajectory. This command will not return output voltages but ChassisSpeeds from the position
   * controllers which need to be converted to module states and put into a velocity PID.
   *
   * <p>Note: The controllers will *not* set the output to zero upon completion of the path this is
   * left to the user, since it is not appropriate for paths with nonstationary endstates.
   *
   * @param trajectory The trajectory to follow.
   * @param poseSupplier A function that supplies the robot pose - use one of the odometry classes
   *     to provide this.
   * @param xController The Trajectory Tracker PID controller for the robot's x position.
   * @param yController The Trajectory Tracker PID controller for the robot's y position.
   * @param rotationController The Trajectory Tracker PID controller for angle for the robot.
   * @param outputChassisSpeeds The field relative chassis speeds output consumer.
   * @param useAllianceColor Should the path states be automatically transformed based on alliance
   *     color? In order for this to work properly, you MUST create your path on the blue side of
   *     the field.
   * @param isFinished The Boolean Supplier for if the trajectory should end.
   * @param endPose The Pose2d for the trajectory's ending position.
   * @param driveSubsystem The swerve drive subsystem.   
   */
  public RealTimePPSwerveControllerCommand(
      PathPlannerTrajectory trajectory,
      Supplier<Pose2d> poseSupplier,
      PIDController xController,
      PIDController yController,
      PIDController rotationController,
      Consumer<ChassisSpeeds> outputChassisSpeeds,
      boolean useAllianceColor,
      BooleanSupplier isFinished,
      Pose2d endPose,
      DriveSubsystem driveSubsystem) {
    this.trajectory = trajectory;
    this.poseSupplier = poseSupplier;
    this.controller = new PPHolonomicDriveController(xController, yController, rotationController);
    this.outputChassisSpeeds = outputChassisSpeeds;
    this.outputModuleStates = null;
    this.kinematics = null;
    this.useKinematics = false;
    this.useAllianceColor = useAllianceColor;
    this.isFinished = isFinished;
    this.endPose = endPose;
    this.driveSubsystem = driveSubsystem;

    addRequirements(driveSubsystem);

    if (useAllianceColor && trajectory.fromGUI && trajectory.getInitialPose().getX() > 8.27) {
      DriverStation.reportWarning(
          "You have constructed a path following command that will automatically transform path states depending"
              + " on the alliance color, however, it appears this path was created on the red side of the field"
              + " instead of the blue side. This is likely an error.",
          false);
    }
  }

  /**
   * Constructs a new RealTimePPSwerveControllerCommand that when executed will follow the provided
   * trajectory. This command will not return output voltages but ChassisSpeeds from the position
   * controllers which need to be converted to module states and put into a velocity PID.
   *
   * <p>Note: The controllers will *not* set the output to zero upon completion of the path this is
   * left to the user, since it is not appropriate for paths with nonstationary endstates.
   *
   * @param trajectory The trajectory to follow.
   * @param poseSupplier A function that supplies the robot pose - use one of the odometry classes
   *     to provide this.
   * @param xController The Trajectory Tracker PID controller for the robot's x position.
   * @param yController The Trajectory Tracker PID controller for the robot's y position.
   * @param rotationController The Trajectory Tracker PID controller for angle for the robot.
   * @param outputChassisSpeeds The field relative chassis speeds output consumer.
   * @param isFinished The Boolean Supplier for if the trajectory should end.
   * @param endPose The Pose2d for the trajectory's ending position.
   * @param driveSubsystem The swerve drive subsystem.  
   */
  public RealTimePPSwerveControllerCommand(
      PathPlannerTrajectory trajectory,
      Supplier<Pose2d> poseSupplier,
      PIDController xController,
      PIDController yController,
      PIDController rotationController,
      Consumer<ChassisSpeeds> outputChassisSpeeds,
      BooleanSupplier isFinished,
      Pose2d endPose,
      DriveSubsystem driveSubsystem) {
    this(
        trajectory,
        poseSupplier,
        xController,
        yController,
        rotationController,
        outputChassisSpeeds,
        false,
        isFinished,
        endPose,
        driveSubsystem);
  }

  /**
   * Constructs a new RealTimePPSwerveControllerCommand that when executed will follow the provided
   * trajectory. This command will not return output voltages but rather raw module states from the
   * position controllers which need to be put into a velocity PID.
   *
   * <p>Note: The controllers will *not* set the output to zero upon completion of the path- this is
   * left to the user, since it is not appropriate for paths with nonstationary endstates.
   *
   * @param trajectory The trajectory to follow.
   * @param poseSupplier A function that supplies the robot pose - use one of the odometry classes
   *     to provide this.
   * @param kinematics The kinematics for the robot drivetrain.
   * @param xController The Trajectory Tracker PID controller for the robot's x position.
   * @param yController The Trajectory Tracker PID controller for the robot's y position.
   * @param rotationController The Trajectory Tracker PID controller for angle for the robot.
   * @param outputModuleStates The raw output module states from the position controllers.
   * @param useAllianceColor Should the path states be automatically transformed based on alliance
   *     color? In order for this to work properly, you MUST create your path on the blue side of
   *     the field.
   * @param isFinished The Boolean Supplier for if the trajectory should end.
   * @param endPose The Pose2d for the trajectory's ending position.
   * @param driveSubsystem The swerve drive subsystem.  
   */
  public RealTimePPSwerveControllerCommand(
      PathPlannerTrajectory trajectory,
      Supplier<Pose2d> poseSupplier,
      SwerveDriveKinematics kinematics,
      PIDController xController,
      PIDController yController,
      PIDController rotationController,
      Consumer<SwerveModuleState[]> outputModuleStates,
      boolean useAllianceColor,
      BooleanSupplier isFinished,
      Pose2d endPose,
      DriveSubsystem driveSubsystem) {
    this.trajectory = trajectory;
    this.poseSupplier = poseSupplier;
    this.kinematics = kinematics;
    this.controller = new PPHolonomicDriveController(xController, yController, rotationController);
    this.outputModuleStates = outputModuleStates;
    this.outputChassisSpeeds = null;
    this.useKinematics = true;
    this.useAllianceColor = useAllianceColor;
    this.isFinished = isFinished;
    this.endPose = endPose;
    this.driveSubsystem = driveSubsystem;

    addRequirements(driveSubsystem);

    if (useAllianceColor && trajectory.fromGUI && trajectory.getInitialPose().getX() > 8.27) {
      DriverStation.reportWarning(
          "You have constructed a path following command that will automatically transform path states depending"
              + " on the alliance color, however, it appears this path was created on the red side of the field"
              + " instead of the blue side. This is likely an error.",
          false);
    }
  }

  /**
   * Constructs a new RealTimePPSwerveControllerCommand that when executed will follow the provided
   * trajectory. This command will not return output voltages but rather raw module states from the
   * position controllers which need to be put into a velocity PID.
   *
   * <p>Note: The controllers will *not* set the output to zero upon completion of the path- this is
   * left to the user, since it is not appropriate for paths with nonstationary endstates.
   *
   * @param trajectory The trajectory to follow.
   * @param poseSupplier A function that supplies the robot pose - use one of the odometry classes
   *     to provide this.
   * @param kinematics The kinematics for the robot drivetrain.
   * @param xController The Trajectory Tracker PID controller for the robot's x position.
   * @param yController The Trajectory Tracker PID controller for the robot's y position.
   * @param rotationController The Trajectory Tracker PID controller for angle for the robot.
   * @param outputModuleStates The raw output module states from the position controllers.
   * @param isFinished The Boolean Supplier for if the trajectory should end.
   * @param endPose The Pose2d for the trajectory's ending position.
   * @param driveSubsystem The swerve drive subsystem.  
   */
  public RealTimePPSwerveControllerCommand(
      PathPlannerTrajectory trajectory,
      Supplier<Pose2d> poseSupplier,
      SwerveDriveKinematics kinematics,
      PIDController xController,
      PIDController yController,
      PIDController rotationController,
      Consumer<SwerveModuleState[]> outputModuleStates,
      BooleanSupplier isFinished,
      Pose2d endPose,
      DriveSubsystem driveSubsystem) {
    this(
        trajectory,
        poseSupplier,
        kinematics,
        xController,
        yController,
        rotationController,
        outputModuleStates,
        false,
        isFinished,
        endPose,
        driveSubsystem);
  }

  @Override
  public void initialize() {
    if (useAllianceColor && trajectory.fromGUI) {
      transformedTrajectory =
          PathPlannerTrajectory.transformTrajectoryForAlliance(
              trajectory, DriverStation.getAlliance());
    } else {
      transformedTrajectory = trajectory;
    }

    if (logActiveTrajectory != null) {
      logActiveTrajectory.accept(transformedTrajectory);
    }

    timer.reset();
    timer.start();

    PathPlannerServer.sendActivePath(transformedTrajectory.getStates());
  }

  @Override
  public void execute() {
    double currentTime = this.timer.get();
    PathPlannerState desiredState = (PathPlannerState) transformedTrajectory.sample(currentTime);

    Pose2d currentPose = this.poseSupplier.get();

    PathPlannerServer.sendPathFollowingData(
        new Pose2d(desiredState.poseMeters.getTranslation(), desiredState.holonomicRotation),
        currentPose);

    ChassisSpeeds targetChassisSpeeds = this.controller.calculate(currentPose, desiredState);

    if (this.useKinematics) {
      SwerveModuleState[] targetModuleStates =
          this.kinematics.toSwerveModuleStates(targetChassisSpeeds);

      this.outputModuleStates.accept(targetModuleStates);
    } else {
      this.outputChassisSpeeds.accept(targetChassisSpeeds);
    }

    if (logTargetPose != null) {
      logTargetPose.accept(
          new Pose2d(desiredState.poseMeters.getTranslation(), desiredState.holonomicRotation));
    }

    if (logError != null) {
      logError.accept(
          currentPose.getTranslation().minus(desiredState.poseMeters.getTranslation()),
          currentPose.getRotation().minus(desiredState.holonomicRotation));
    }

    if (logSetpoint != null) {
      logSetpoint.accept(targetChassisSpeeds);
    }
  }

  @Override
  public void end(boolean interrupted) {
    this.timer.stop();

    if (interrupted
        || Math.abs(transformedTrajectory.getEndState().velocityMetersPerSecond) < 0.1) {
      if (useKinematics) {
        this.outputModuleStates.accept(
            this.kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0)));
      } else {
        this.outputChassisSpeeds.accept(new ChassisSpeeds());
      }
    }
  }

  @Override
  public boolean isFinished() {
    // Ends the command if the button being held down is released or the robot reaches the end point
    return isFinished.getAsBoolean() || (Math.abs(endPose.getX() - driveSubsystem.getPose().getX()) < .05 &&
      Math.abs(endPose.getY() - driveSubsystem.getPose().getY()) < .05);
  }

  private static void defaultLogError(Translation2d translationError, Rotation2d rotationError) {}

  /**
   * Set custom logging callbacks for this command to use instead of the default configuration of
   * pushing values to SmartDashboard
   *
   * @param logActiveTrajectory Consumer that accepts a PathPlannerTrajectory representing the
   *     active path. This will be called whenever a RealTimePPSwerveControllerCommand starts
   * @param logTargetPose Consumer that accepts a Pose2d representing the target pose while path
   *     following
   * @param logSetpoint Consumer that accepts a ChassisSpeeds object representing the setpoint
   *     speeds
   * @param logError BiConsumer that accepts a Translation2d and Rotation2d representing the error
   *     while path following
   */
  public static void setLoggingCallbacks(
      Consumer<PathPlannerTrajectory> logActiveTrajectory,
      Consumer<Pose2d> logTargetPose,
      Consumer<ChassisSpeeds> logSetpoint,
      BiConsumer<Translation2d, Rotation2d> logError) {
    RealTimePPSwerveControllerCommand.logActiveTrajectory = logActiveTrajectory;
    RealTimePPSwerveControllerCommand.logTargetPose = logTargetPose;
    RealTimePPSwerveControllerCommand.logSetpoint = logSetpoint;
    RealTimePPSwerveControllerCommand.logError = logError;
  }
}
