// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2025.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import team3647.frc2025.commands.DrivetrainCommands;
import team3647.frc2025.constants.DrivetrainConstants;
import team3647.frc2025.subsystems.Drivetrain;
import team3647.lib.inputs.Joysticks;

public class RobotContainer {
  
  private final Drivetrain drivetrain = new Drivetrain(
    DrivetrainConstants.leftMotor, DrivetrainConstants.rightMotor, DrivetrainConstants.gyro);
  DrivetrainCommands driveCommand = new DrivetrainCommands(drivetrain);
  private final Joysticks mainController = new Joysticks(0);



  public RobotContainer() {
    CommandScheduler.getInstance().registerSubsystem(drivetrain);
    drivetrain.setDefaultCommand(driveCommand.drive(mainController::getLeftStickY,mainController::getRightStickX));

    mainController.buttonA.onTrue(driveCommand.turnWheel());

    
    configureBindings();
  }

  private void configureBindings() {}

  
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
