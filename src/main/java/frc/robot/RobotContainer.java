// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {

  private final SendableChooser<Command> autoChooser;

 
  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser",autoChooser);

    //driveSubsystem.setDefaultCommand(new ArcadeDriveCommand(driveSubsystem, () -> m_driverController.getLeftY() * -1, () -> m_driverController.getLeftX()));

    configureBindings();
  }

  private void configureBindings() {
    
  }


  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
