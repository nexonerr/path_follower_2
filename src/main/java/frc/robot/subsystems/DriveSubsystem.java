// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

  public SwerveModule[] modules;

  private final AHRS gyro = new AHRS();

  private final SwerveDriveOdometry odometry;

  private Field2d field = new Field2d();

  public DriveSubsystem() {

    modules = new SwerveModule[]{
      new SwerveModule(),
      new SwerveModule(),
      new SwerveModule(),
      new SwerveModule()
    };

    odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, gyro.getRotation2d(), getPositions());

      //Take this part
    AutoBuilder.configureHolonomic(
            this::getPose, 
            this::resetPose, 
            this::getCurrentSpeeds, 
            this::driveRobotRelative, 
            new HolonomicPathFollowerConfig(
                    new PIDConstants(DriveConstants.translationP, DriveConstants.translationI, DriveConstants.translationD),
                    new PIDConstants(DriveConstants.rotP, DriveConstants.rotI, DriveConstants.rotD),
                    DriveConstants.maxModuleSpeed,
                    DriveConstants.driveBaseRadius,
                    new ReplanningConfig()
            ), 
            () -> {
              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this
    );
  }

  @Override
  public void periodic() {
    odometry.update(gyro.getRotation2d(), getPositions());
    field.setRobotPose(odometry.getPoseMeters());
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose){
    odometry.resetPosition(Rotation2d.fromDegrees(getHeading()), getPositions(), pose);
  }

  public ChassisSpeeds getCurrentSpeeds(){
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

   public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    SwerveModuleState[] targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
    setStates(targetStates);
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }

  public void setStates(SwerveModuleState[] targetStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, DriveConstants.maxModuleSpeed);

    for (int i = 0; i < modules.length; i++) {
      modules[i].setTargetState(targetStates[i]);
    }
  }

  public double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
}

//this should be changed to whatever swerve module class we're using
class SwerveModule {
    private SwerveModulePosition currentPosition = new SwerveModulePosition();
    private SwerveModuleState currentState = new SwerveModuleState();

    public SwerveModulePosition getPosition() {
      return currentPosition;
    }

    public SwerveModuleState getState() {
      return currentState;
    }

    public void setTargetState(SwerveModuleState targetState) {
      // Optimize the state
      currentState = SwerveModuleState.optimize(targetState, currentState.angle);

      currentPosition = new SwerveModulePosition(currentPosition.distanceMeters + (currentState.speedMetersPerSecond * 0.02), currentState.angle);
    }
  }

