// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class PoseEstimationSubsystem extends SubsystemBase {
  
  private final DriveSubsystem driveSubsystem;
  private final SwerveDrivePoseEstimator poseEstimator;
  private final NetworkTable networkTable; 
  private final NetworkTableEntry botPoseNetworkTableEntry;
  private final NetworkTableEntry jsonDumpNetworkTableEntry;

  /**
   * Standard deviations of model states. Increase these numbers to trust your model's state estimates less. This
   * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then meters.
   */
  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
  
  /**
   * Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
   * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
   */
  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

  private double lastTimeStampSeconds = 0;

  public PoseEstimationSubsystem(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;

    poseEstimator = new SwerveDrivePoseEstimator(
      DriveConstants.driveKinematics,
      driveSubsystem.getRotation2d(),
      driveSubsystem.getModulePositions(),
      new Pose2d(),  // FIXME: This is the pose2d for where you start on the field, might need to change
      stateStdDevs,
      visionMeasurementStdDevs
    );

    networkTable = NetworkTableInstance.getDefault().getTable("limelight-tigers");
    botPoseNetworkTableEntry = networkTable.getEntry("botpose");
    jsonDumpNetworkTableEntry = networkTable.getEntry("json");
  }

  @Override
  public void periodic() {
    // Gets the robot's pose from the network table with an empty array as the default value
    double[] botPose = botPoseNetworkTableEntry.getDoubleArray(new double[]{});
    String jsonDump = jsonDumpNetworkTableEntry.getString("{}");  

    double currentTimeStampSeconds = lastTimeStampSeconds;
    // Attempts to get the time stamp for when the robot pose was calculated
    try {
      ObjectMapper mapper = new ObjectMapper();
      JsonNode jsonNodeData = mapper.readTree(jsonDump);
      double tsValue = jsonNodeData.path("Results").path("ts").asDouble();
      SmartDashboard.putNumber("tsValue", timeStampValue);
      if (timeStampValue != 0) {
        // Converts from milleseconds to seconds
        currentTimeStampSeconds = timeStampValue / 1000;
      }
    } catch (JsonProcessingException e) {
      SmartDashboard.putString("Json Parsing Error", e.getStackTrace());
    }

    // Updates the pose estimator's position if limelight position data was recieved with a new time stamp
    if (botPose.length != 0 && currentTimeStampSeconds > lastTimeStampSeconds) {
      double robotX = botPose[0] + 8.28; // TODO: Get precise field measurements
      double robotY = botPose[1] + 4;
      Rotation2d robotRotation = Rotation2d.fromDegrees(botPose[5]);
      Pose2d limelightVisionMeasurement = new Pose2d(robotX, robotY, robotRotation);
      poseEstimator.addVisionMeasurement(limelightVisionMeasurement, currentTimeStampSeconds);
      SmartDashboard.putString("Limelight Pose", limelightVisionMeasurement.toString());
    }

    lastTimeStampSeconds = currentTimeStampSeconds;

    // Uses the swerve's sensors to update the pose estimator
    poseEstimator.update(
      driveSubsystem.getRotation2d(),
      driveSubsystem.getModulePositions()
    );

    // Updates the odometry to the pose estimator's pose
    driveSubsystem.resetOdometry(getPose());

    SmartDashboard.putString("Estimated Pose", getPose().toString());
    SmartDashboard.putString("Odometry Pose", driveSubsystem.getPose().toString());
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the pose estimator to the specified pose. Keeps the current
   * rotation and module positions.
   * @param newPose The Pose2d object
   */
  public void setPose(Pose2d newPose) {
    poseEstimator.resetPosition(
      driveSubsystem.getRotation2d(),
      driveSubsystem.getModulePositions(),
      newPose
    );
  }

  /**
   * Resets the position on the field to 0,0 0-degrees, with forward being downfield. 
   * This resets what "forward" is for field oriented driving.
   */
  public void resetFieldPosition() {
    setPose(new Pose2d());
  }

}
