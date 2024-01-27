// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
<<<<<<< Updated upstream
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
=======
  private XboxController driverController;
  private Drive swerve;

  private AutoMode auto;
  private PathPlannerTrajectory trajectory = PathPlannerPath.fromPathFile("New New Path").getTrajectory(new ChassisSpeeds(), new Rotation2d());


>>>>>>> Stashed changes
  @Override
  public void robotInit() {}

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
<<<<<<< Updated upstream
  public void teleopPeriodic() {}
=======
  public void teleopPeriodic() {
    if(driverController.getYButtonPressed()) {
      swerve.resetHeading();
    }
    if(swerve.getState() != DriveState.LOCK && driverController.getXButton()) {
      swerve.setWantedState(DriveState.LOCK);
    } else if(swerve.getState() != DriveState.TELEOP_APRILTAG_TRACK && driverController.getRightBumper()) {
      swerve.setWantedState(DriveState.TELEOP_APRILTAG_TRACK);
    } else if(swerve.getState() != DriveState.TELEOP_DRIVE) {
      swerve.setWantedState(DriveState.TELEOP_DRIVE);
    }
    swerve.update();
  }
>>>>>>> Stashed changes

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
