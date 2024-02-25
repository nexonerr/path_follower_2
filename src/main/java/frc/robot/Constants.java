// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {
  public static final class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static final class DriveConstants {

    //PID
    //Swerve Auto Start
    public static final double rotP = 5.0;
    public static final double rotI = 0.0;
    public static final double rotD = 0.0;

    public static final double translationP = 5.0;
    public static final double translationI = 0.0;
    public static final double translationD = 0.0;

    public static final double maxModuleSpeed = 5.0; // in m/s
    public static final double driveBaseRadius = 0.5; // in m
    //Swerve Auto End

    //this needs to be configured with the position of the swerve modules relative to the center of the drivebase
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(new Translation2d(-0.25,-0.25),new Translation2d(-0.25,0.25),new Translation2d(0.25,0.25),new Translation2d(0.25,-0.25));

    public static final int kEncoderCPR = 1024;
    public static final double kWheelDiameterMeters = 0.15;
    public static final double kEncoderDistancePerPulse = (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

    public static final boolean kGyroReversed = false;
  }
}
