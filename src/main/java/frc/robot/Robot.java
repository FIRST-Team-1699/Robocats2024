// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.team1699.Constants.InputConstants;
import frc.team1699.lib.auto.modes.AutoMode;
import frc.team1699.lib.auto.modes.FourPieceCenter;
import frc.team1699.subsystems.Climber;
import frc.team1699.subsystems.Drive;
import frc.team1699.subsystems.Indexer;
import frc.team1699.subsystems.Intake;
import frc.team1699.subsystems.Manipulator;
import frc.team1699.subsystems.Pivoter;
import frc.team1699.subsystems.Shooter;
import frc.team1699.subsystems.Climber.ClimbStates;
import frc.team1699.subsystems.Drive.DriveState;
import frc.team1699.subsystems.Indexer.IndexStates;
import frc.team1699.subsystems.Intake.IntakeStates;
import frc.team1699.subsystems.Manipulator.ManipulatorStates;

public class Robot extends TimedRobot {

  private XboxController driverController, operatorController;
  private Drive swerve;
  private Manipulator manipulator;
  private Climber climber;
  private AutoMode auto;

  @Override
  public void robotInit() {
    driverController = new XboxController(InputConstants.kDriverControllerPort);
    operatorController = new XboxController(InputConstants.kOperatorControllerPort);
    swerve = new Drive(driverController);
    manipulator = new Manipulator();
    climber = new Climber();
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
    // auto = new FourPieceCenter(manipulator, swerve);
    // auto.initialize();
    climber.setWantedState(ClimbStates.RETRACTING);
  }

  @Override
  public void autonomousPeriodic() {
    if(auto.isFinished()) {
      auto.finish();
    } else {
      auto.run();
    }
    swerve.update();
    climber.update();
  }

  @Override
  public void teleopInit() {
    manipulator.setWantedState(ManipulatorStates.IDLE);
  }

  @Override
  public void teleopPeriodic() {
    if(driverController.getYButtonPressed()) {
      swerve.resetHeading();
    }

    // if(driverController.getLeftTriggerAxis() > 0.1) {
    //   manipulator.setWantedState(ManipulatorStates.OUTTAKING); 
    // } else if(driverController.getRightTriggerAxis() > 0.1) {
    //   manipulator.setWantedState(ManipulatorStates.INTAKING);
    // } else {
    //   manipulator.setWantedState(ManipulatorStates.IDLE);
    // }

    if(operatorController.getLeftTriggerAxis() > 0.1) {
      manipulator.setWantedState(ManipulatorStates.OUTTAKING);
    } else if(operatorController.getRightTriggerAxis() > 0.1) {
      manipulator.setWantedState(ManipulatorStates.INTAKING);
    } else if(operatorController.getRightBumper()) {
      manipulator.setWantedState(ManipulatorStates.SPEAKER_SHOOT);
    } else if(operatorController.getLeftBumper()) {
      manipulator.setWantedState(ManipulatorStates.AMP_SHOOT);
    } else {
      manipulator.setWantedState(ManipulatorStates.IDLE);
    }

    if(swerve.getState() != DriveState.LOCK && driverController.getXButton()) {
      swerve.setWantedState(DriveState.LOCK);
    } else if(swerve.getState() != DriveState.TELEOP_SPEAKER_TRACK && driverController.getRightBumper()) {
      swerve.setWantedState(DriveState.TELEOP_SPEAKER_TRACK);
    } else if(swerve.getState() != DriveState.TELEOP_DRIVE) {
      swerve.setWantedState(DriveState.TELEOP_DRIVE);
    }

    // if(operatorController.getRightBumperPressed()) {
    //   shooter.setSpeed(5000);
    // } else if(operatorController.getLeftBumperPressed()) {
    //   shooter.setSpeed(0);
    // }

    if(operatorController.getPOV() == 0) {
      climber.setWantedState(ClimbStates.EXTENDING);
    } else if(operatorController.getPOV() == 180) {
      climber.setWantedState(ClimbStates.RETRACTING);
    }

    // if(operatorController.getAButton()) {
    //   pivot.setAngle(45);
    // }

    // if(operatorController.getXButton()) {
    //   pivot.setAngle(60);
    // }

    swerve.update();
    manipulator.update();
    climber.update();
  }

  @Override
  public void disabledInit() {

  }

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
