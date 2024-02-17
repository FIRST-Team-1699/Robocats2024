// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.team1699.Constants.InputConstants;
import frc.team1699.lib.auto.modes.AutoMode;
import frc.team1699.lib.auto.modes.ThreeNoteIntakeTest;
import frc.team1699.lib.sensors.BeamBreak;
import frc.team1699.subsystems.Climber;
import frc.team1699.subsystems.Drive;
import frc.team1699.subsystems.Intake;
import frc.team1699.subsystems.Pivoter;
import frc.team1699.subsystems.Climber.ClimbStates;
import frc.team1699.subsystems.Drive.DriveState;
import frc.team1699.subsystems.Intake.IntakeStates;

public class Robot extends TimedRobot {

  private XboxController driverController, operatorController;
  private Drive swerve;
  private Intake intake;
  // private Pivoter pivot;
  private Climber climber;

  private AutoMode auto;

  @Override
  public void robotInit() {
    driverController = new XboxController(InputConstants.kDriverControllerPort);
    operatorController = new XboxController(InputConstants.kOperatorControllerPort);
    swerve = new Drive(driverController);
    intake = new Intake();
    // pivot = new Pivoter();
    climber = new Climber();
  }

  @Override
  public void robotPeriodic() {
    // pivot.printEncoderValue();
  }

  @Override
  public void autonomousInit() {
    auto = new ThreeNoteIntakeTest(intake, swerve);
    auto.initialize();
    climber.setWantedState(ClimbStates.DOWN);
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
    intake.setWantedState(IntakeStates.IDLE);
  }

  @Override
  public void teleopPeriodic() {
    if(driverController.getYButtonPressed()) {
      swerve.resetHeading();
    }

    if(driverController.getLeftTriggerAxis() > 0.1) {
      intake.setWantedState(IntakeStates.REVERSING); 
    } else if(driverController.getRightTriggerAxis() > 0.1) {
      intake.setWantedState(IntakeStates.INTAKING);
    } else {
      intake.setWantedState(IntakeStates.IDLE);
    }

    if(swerve.getState() != DriveState.LOCK && driverController.getXButton()) {
      swerve.setWantedState(DriveState.LOCK);
    } else if(swerve.getState() != DriveState.TELEOP_SPEAKER_TRACK && driverController.getRightBumper()) {
      swerve.setWantedState(DriveState.TELEOP_SPEAKER_TRACK);
    } else if(swerve.getState() != DriveState.TELEOP_DRIVE) {
      swerve.setWantedState(DriveState.TELEOP_DRIVE);
    }

    if(operatorController.getLeftTriggerAxis() >= .1) {
      climber.setWantedState(ClimbStates.EXTENDING);
    } else if(operatorController.getRightTriggerAxis() >= .1) {
      climber.setWantedState(ClimbStates.RETRACTING);
    }
    swerve.update();
    intake.update();
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
