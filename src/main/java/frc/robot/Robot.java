// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.team1699.Constants.InputConstants;
import frc.team1699.lib.auto.modes.AutoMode;
import frc.team1699.lib.auto.modes.OptimFourPiece;
import frc.team1699.lib.auto.modes.RedOnePieceEscape;
import frc.team1699.lib.leds.LEDController;
import frc.team1699.lib.leds.colors.Blue;
import frc.team1699.lib.leds.colors.Green;
import frc.team1699.lib.leds.colors.Purple;
import frc.team1699.lib.leds.colors.Red;
import frc.team1699.lib.leds.colors.Yellow;
import frc.team1699.subsystems.Climber;
import frc.team1699.subsystems.Drive;
import frc.team1699.subsystems.Manipulator;
import frc.team1699.subsystems.Vision;
import frc.team1699.subsystems.Drive.DriveState;
import frc.team1699.subsystems.Manipulator.ManipulatorStates;

public class Robot extends TimedRobot {

  private XboxController driverController, operatorController;
  private Drive swerve;
  private Manipulator manipulator;
  private Climber climber;
  private AutoMode auto;
  private LEDController ledController;
  // private boolean headingIsReady;

  @Override
  public void robotInit() {
    driverController = new XboxController(InputConstants.kDriverControllerPort);
    operatorController = new XboxController(InputConstants.kOperatorControllerPort);
    swerve = new Drive(driverController); 
    manipulator = new Manipulator();
    climber = new Climber();
    CameraServer.startAutomaticCapture();
    ledController = new LEDController(74, 1);
    ledController.start();
    ledController.solidColor(new Blue());
    // headingIsReady = false;
  }

  @Override
  public void robotPeriodic() {
    // if(!headingIsReady && DriverStation.getAlliance().isPresent()) {
    //   swerve.resetHeading();
    //   headingIsReady = true;
    // }
  }

  @Override
  public void autonomousInit() {
    auto = new RedOnePieceEscape(manipulator, swerve);
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
    manipulator.update();
  }

  @Override
  public void teleopInit() {
    manipulator.setWantedState(ManipulatorStates.IDLE);
  }

  @Override
  public void teleopPeriodic() {
    if(operatorController.getRightBumper()) {
      manipulator.setWantedState(ManipulatorStates.SHOOTING);
      driverController.setRumble(RumbleType.kBothRumble, 1);
      operatorController.setRumble(RumbleType.kBothRumble, 1);
    } else if(operatorController.getLeftTriggerAxis() > 0.1) {
      manipulator.setWantedState(ManipulatorStates.OUTTAKING);
    } else if(operatorController.getRightTriggerAxis() > 0.1) {
      manipulator.setWantedState(ManipulatorStates.INTAKING);
    } else if(operatorController.getBButtonPressed()) {
      manipulator.setWantedState(ManipulatorStates.SPEAKER_SUB_SHOOT);
    } else if(operatorController.getYButtonPressed()) {
      manipulator.setWantedState(ManipulatorStates.SPEAKER_GOOFY_SHOOT);
    } else if(operatorController.getAButtonPressed()) {
      manipulator.setWantedState(ManipulatorStates.AMP_SHOOT);
    } else if(operatorController.getXButtonPressed()) {
      manipulator.setWantedState(ManipulatorStates.TRAP_SHOOT);
    } else if(driverController.getRightBumper()) {
      manipulator.setWantedState(ManipulatorStates.SPEAKER_LL_SHOOT);
      if(!Vision.getInstance().hasTargetInView()) {
        if(ledController.getColor().getHue() != Red.redHue) {
          ledController.solidColor(new Red());
        } 
      } else {
        if(swerve.headingAimed() && manipulator.pivotAtPose() && manipulator.shooterAtSpeed()) {
          if(ledController.getColor().getHue() != Green.greenHue) {
            ledController.solidColor(new Green());
          }
        } else {
          if(ledController.getColor().getHue() != Blue.blueHue) {
            ledController.solidColor(new Blue());
          }
        }
      }
    } else {
      manipulator.setWantedState(ManipulatorStates.IDLE);
      driverController.setRumble(RumbleType.kBothRumble, 0);
      operatorController.setRumble(RumbleType.kBothRumble, 0);
      if(ledController.getColor().getHue() != Blue.blueHue) {
        ledController.solidColor(new Blue());
      }
    }

    if(operatorController.getRawButton(7)) {
      ledController.rainbow();
    }

    if(driverController.getYButtonPressed()) {
      swerve.resetHeading();
      if(ledController.getColor().getHue() != Purple.purpleHue) {
        ledController.solidColor(new Purple());

      }
    }

    if(swerve.getState() != DriveState.LOCK && driverController.getXButton()) {
      swerve.setWantedState(DriveState.LOCK);
    } else if(driverController.getRightBumper()) {
      swerve.setWantedState(DriveState.TELEOP_SPEAKER_TRACK);
    } else if(swerve.getState() != DriveState.TELEOP_DRIVE) {
      swerve.setWantedState(DriveState.TELEOP_DRIVE);
    }

    if(operatorController.getPOV() == 0) {
      climber.raise();
    } else if(operatorController.getPOV() == 180) {
      climber.lower();
    }

    swerve.update();
    manipulator.update();
  }

  @Override
  public void disabledInit() {
    driverController.setRumble(RumbleType.kBothRumble, 0);
    operatorController.setRumble(RumbleType.kBothRumble, 0);
  }

  @Override
  public void disabledPeriodic() {
    ledController.alternateColors(new Blue(), new Yellow());
  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {
    if(operatorController.getRightTriggerAxis() > 0.1) {
      climber.slowUp();
    } else if(operatorController.getRightBumper()) {
      climber.slowDown();
    } if(operatorController.getAButtonPressed()) {
      climber.overridePosition();
    }
  }

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
