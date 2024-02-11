// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.team1699.Constants.InputConstants;
import frc.team1699.lib.auto.modes.AutoMode;
<<<<<<< Updated upstream
import frc.team1699.lib.auto.modes.TestTrajectoryMode;
=======
import frc.team1699.lib.auto.modes.ThreeNoteIntakeTest;
>>>>>>> Stashed changes
import frc.team1699.subsystems.Drive;
import frc.team1699.subsystems.Drive.DriveState;

public class Robot extends TimedRobot {

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  private XboxController driverController;
  private Drive swerve;

  private AutoMode auto;

  //private PathPlannerTrajectory trajectory = PathPlannerPath.fromPathFile("TestTrajectory").getTrajectory(new ChassisSpeeds(), new Rotation2d());

  private PathPlannerTrajectory trajectory = PathPlannerPath.fromPathFile("New New Path").getTrajectory(new ChassisSpeeds(), new Rotation2d());

  @Override
  public void robotInit() {
    driverController = new XboxController(InputConstants.kDriverControllerPort);
    swerve = new Drive(driverController);
    swerve.setTrajectory(trajectory);
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
    auto = new TestTrajectoryMode(trajectory, swerve);
    auto.initialize();
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
    if(driverController.getYButtonPressed()) {
      swerve.resetHeading();
    }
  // photonvision-heading
  /*  if(driverController.getXButton()) {
      swerve.setWantedState(DriveState.LOCK);
    } else if(driverController.getRightBumper()) {
      swerve.setWantedState(DriveState.TELEOP_APRILTAG_TRACK);
    } else { */

    if(swerve.getState() != DriveState.LOCK && driverController.getXButton()) {
      swerve.setWantedState(DriveState.LOCK);
    } else if(swerve.getState() != DriveState.TELEOP_APRILTAG_TRACK && driverController.getRightBumper()) {
      swerve.setWantedState(DriveState.TELEOP_APRILTAG_TRACK);
    } else if(swerve.getState() != DriveState.TELEOP_DRIVE) {
//main
      swerve.setWantedState(DriveState.TELEOP_DRIVE);
    }
    swerve.update();
<<<<<<< Updated upstream
=======
    intake.update();
>>>>>>> Stashed changes
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    // manipulator.startOrchestra();
  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
