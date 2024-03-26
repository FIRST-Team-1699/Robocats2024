// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team1699.Constants.InputConstants;
import frc.team1699.Constants.LEDConstants;
import frc.team1699.lib.auto.modes.AutoMode;
import frc.team1699.lib.auto.modes.BlueOnePieceEscape;
import frc.team1699.lib.auto.modes.DoNothing;
import frc.team1699.lib.auto.modes.FivePieceRed;
import frc.team1699.lib.auto.modes.BlueAmpSideFourPiece;
import frc.team1699.lib.auto.modes.OptimThreePiece;
import frc.team1699.lib.auto.modes.OptimizingFivePieceBlue;
import frc.team1699.lib.auto.modes.RedAmpSideFourPiece;
import frc.team1699.lib.auto.modes.RedOnePieceEscape;
import frc.team1699.lib.auto.modes.ShootAndWait;
import frc.team1699.lib.leds.LEDController;
import frc.team1699.lib.leds.LEDController.LEDStates;
import frc.team1699.lib.leds.colors.Blue;
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

  private SendableChooser<String> autoChooser;
  private final String onePiece = "One Piece And Escape";
  private final String fivePiece = "Five Piece";
  private final String fourPiecePodium = "Pod Side Four Piece";
  private final String fourPieceAmp = "Amp Side Four Piece";
  private final String threePiece = "Three Piece";
  private final String doNothing = "Do Nothing";
  private final String scoreAndWait = "Shoot Preload Only";

  private String lastChosenAuto = fivePiece;
  private static boolean initialAutoSet = false;

  @Override
  public void robotInit() {
    driverController = new XboxController(InputConstants.kDriverControllerPort);
    operatorController = new XboxController(InputConstants.kOperatorControllerPort);
    swerve = new Drive(driverController); 
    manipulator = new Manipulator();
    climber = new Climber();
    CameraServer.startAutomaticCapture();
    ledController = new LEDController(LEDConstants.kLEDLength, LEDConstants.kLEDPort, swerve, manipulator);
    ledController.solidColor(new Blue());

    autoChooser = new SendableChooser<String>();
    autoChooser.addOption(onePiece, onePiece);
    autoChooser.addOption(threePiece, threePiece);
    autoChooser.addOption(fourPiecePodium, fourPiecePodium);
    autoChooser.addOption(fivePiece, fivePiece);
    autoChooser.addOption(doNothing, doNothing);
    autoChooser.addOption(scoreAndWait, scoreAndWait);
    autoChooser.setDefaultOption(fourPieceAmp, fourPieceAmp);
    SmartDashboard.putData(autoChooser);
  }

  @Override
  public void robotPeriodic() {
    if(!DriverStation.isDisabled()) {
      ledController.addState(LEDStates.IDLE);
      ledController.update();
    }
  }

  @Override
  public void autonomousInit() {
    auto.initialize();
  }

  @Override
  public void autonomousPeriodic() {
    // ledController.addState(LEDStates.AUTO);
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
      // driverController.setRumble(RumbleType.kBothRumble, 1);
      // operatorController.setRumble(RumbleType.kBothRumble, 1);
    } else if(operatorController.getLeftTriggerAxis() > 0.1) {
      manipulator.setWantedState(ManipulatorStates.OUTTAKING);
    } else if(operatorController.getRightTriggerAxis() > 0.1 && !manipulator.isLoaded()) {
      manipulator.setWantedState(ManipulatorStates.INTAKING);
    } else if(operatorController.getBButtonPressed()) {
      manipulator.setWantedState(ManipulatorStates.SPEAKER_SUB_SHOOT);
    } else if(operatorController.getYButtonPressed()) {
      manipulator.setWantedState(ManipulatorStates.SHUFFLE);
    } else if(operatorController.getAButtonPressed()) {
      manipulator.setWantedState(ManipulatorStates.AMP_SHOOT);
    } else if(operatorController.getXButtonPressed()) {
      manipulator.setWantedState(ManipulatorStates.TRAP_SHOOT);
    } else if(driverController.getRightBumper()) {
      manipulator.setWantedState(ManipulatorStates.SPEAKER_LL_SHOOT);
      ledController.addState(LEDStates.AIMING);
      if(manipulator.pivotAtPose() && manipulator.shooterAtSpeed() && Vision.getInstance().hasTargetInView()) {
        driverController.setRumble(RumbleType.kBothRumble, 1);
        operatorController.setRumble(RumbleType.kBothRumble, 1);
      } else {
        driverController.setRumble(RumbleType.kBothRumble, 0);
        operatorController.setRumble(RumbleType.kBothRumble, 0);
      }
    } else {
      manipulator.setWantedState(ManipulatorStates.IDLE);
      driverController.setRumble(RumbleType.kBothRumble, 0);
      operatorController.setRumble(RumbleType.kBothRumble, 0);
    }

    if(driverController.getYButtonPressed()) {
      swerve.zeroGyro();
    }

    if(operatorController.getRawButton(7)) {
      ledController.addState(LEDStates.AMPLIFY);
    }

    if(swerve.getState() != DriveState.LOCK && driverController.getXButton()) {
      swerve.setWantedState(DriveState.LOCK);
    } else if(driverController.getRightBumper()) {
      if(Vision.getInstance().hasTargetInView()) {
        swerve.setWantedState(DriveState.TELEOP_SPEAKER_TRACK);
      }
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
    // driverController.setRumble(RumbleType.kBothRumble, 0);
    // operatorController.setRumble(RumbleType.kBothRumble, 0);
    ledController.addState(LEDStates.AUTO);
  }

  @Override
  public void disabledPeriodic() {
    if((autoChooser.getSelected() != lastChosenAuto || !initialAutoSet) && DriverStation.getAlliance().isPresent()) {
      lastChosenAuto = autoChooser.getSelected();
      initialAutoSet = true;
      switch(autoChooser.getSelected()) {
        case scoreAndWait:
          auto = new ShootAndWait(manipulator, swerve);
          break;
        case doNothing:
          auto = new DoNothing();
          break;
        case onePiece:
          if(DriverStation.getAlliance().get() == Alliance.Red) {
            auto = new RedOnePieceEscape(manipulator, swerve);
          } else {
            auto = new BlueOnePieceEscape(manipulator, swerve);
          }
          break;
        case threePiece:
          auto = new OptimThreePiece(manipulator, swerve);
        case fourPieceAmp:
          if(DriverStation.getAlliance().get() == Alliance.Red) {
            auto = new RedAmpSideFourPiece(manipulator, swerve);
          } else {
            auto = new BlueAmpSideFourPiece(manipulator, swerve);
          }
          break;
        case fourPiecePodium:
          if(DriverStation.getAlliance().get() == Alliance.Red) {
            auto = new BlueAmpSideFourPiece(manipulator, swerve);
          } else {
            auto = new RedAmpSideFourPiece(manipulator, swerve);
          }
          break;
        case fivePiece:
          if(DriverStation.getAlliance().get() == Alliance.Red) {
            auto = new FivePieceRed(manipulator, swerve);
          } else {
            auto = new OptimizingFivePieceBlue(manipulator, swerve);
          }
          break;
        default:
          break;
      }
    }
    ledController.update();
  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {
    if(operatorController.getYButton()) {
      climber.slowUp();
    } else if(operatorController.getAButtonPressed()) {
      climber.bringToZero();
    }
    climber.beamBreakCheck();
  }

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
