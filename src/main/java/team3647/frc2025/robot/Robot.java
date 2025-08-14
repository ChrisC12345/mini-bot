// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2025.robot;

import java.util.Optional;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import choreo.Choreo;
import choreo.trajectory.DifferentialSample;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import team3647.frc2025.constants.DrivetrainConstants;
import team3647.frc2025.subsystems.Drivetrain;

/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends LoggedRobot {
  @SuppressWarnings("unused")
  private final RobotContainer robotContainer = new RobotContainer();

  private final Optional<Trajectory<DifferentialSample>> trajectory = Choreo.loadTrajectory("myTrajectory");
      
  private final Drivetrain drivetrain = new Drivetrain(
      DrivetrainConstants.leftMotor, DrivetrainConstants.rightMotor, DrivetrainConstants.gyro);
  private final Timer timer = new Timer();



  /** Called once at the beginning of the robot program. */
  public Robot() {

    if (isReal()) {
        Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs")); // Log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    } else {
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
        Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }

    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopPeriodic() {
  }


}
