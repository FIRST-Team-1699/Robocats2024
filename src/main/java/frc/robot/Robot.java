// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.team1699.Constants.InputConstants;
import frc.team1699.lib.auto.modes.AutoMode;
import frc.team1699.lib.auto.modes.TestTrajectoryMode;
import frc.team1699.subsystems.Drive;
import frc.team1699.subsystems.Drive.DriveState;

public class Robot extends TimedRobot {
  private XboxController driverController;
  private Drive swerve;

  private AutoMode auto;

  @Override
  public void robotInit() {
    driverController = new XboxController(InputConstants.kDriverControllerPort);
    swerve = new Drive(driverController);
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
    auto = new TestTrajectoryMode(PathPlannerPath.fromPathFile("TestTrajectoryOne").getTrajectory(new ChassisSpeeds(), swerve.getHeading()), swerve);
  }

  @Override
  public void autonomousPeriodic() {
    if(auto.isFinished()) {
      auto.finish();
    } else {
      auto.run();
    }
    swerve.update();
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    if(driverController.getXButton()) {
      swerve.setWantedState(DriveState.LOCK);
    } else if(driverController.getPOV() != -1) {
      swerve.setWantedState(DriveState.TELEOP_HEADING_LOCKED);
    } else {
      swerve.setWantedState(DriveState.TELEOP_DRIVE);
    }
    swerve.update();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
