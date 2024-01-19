// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {
  private XboxController driverController;
  private Drive swerve;

  @Override
  public void robotInit() {
    driverController = new XboxController(InputConstants.kDriveControllerPort);
    swerve = new Drive(driverController);
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {
    swerve.update();
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    if(controller.getXButton()) {
      swerve.setWantedState(DriveState.LOCK);
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
