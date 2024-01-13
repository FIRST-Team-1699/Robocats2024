// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.team1699.Constants;
import frc.robot.team1699.subsystems.Drivetrain;
import frc.robot.team1699.subsystems.IntakeHopper;
import frc.robot.team1699.subsystems.IntakeHopper.IntakeHopperState;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  private Drivetrain drive = new Drivetrain();
  private IntakeHopper intakeHopper = new IntakeHopper();
  private Compressor compressor = new Compressor(1, PneumaticsModuleType.CTREPCM);
  private XboxController controller = new XboxController(Constants.kDriverID);

  @Override
  public void robotInit() {
    compressor.enableDigital();
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    drive.setIdleMode(NeutralMode.Brake);
  }

  @Override
  public void teleopPeriodic() {
    drive.runArcadeDrive(controller.getLeftY(), -controller.getRightX());
    if(controller.getRightTriggerAxis() > 0) {
      intakeHopper.setWantedState(IntakeHopperState.INTAKING);
    } else if(controller.getLeftTriggerAxis() > 0) {
      intakeHopper.setWantedState(IntakeHopperState.OUTTAKING);
    } else {
      intakeHopper.setWantedState(IntakeHopperState.STORED);
    }

    intakeHopper.update();
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
