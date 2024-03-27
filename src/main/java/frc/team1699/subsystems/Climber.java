package frc.team1699.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.team1699.Constants.ClimberConstants;
import frc.team1699.lib.sensors.BeamBreak;;

public class Climber {
    private CANSparkMax portWinch, starWinch;
    private SparkPIDController portController, starController;
    private RelativeEncoder portEncoder, starEncoder;
    private BeamBreak portBeamBreak, starBeamBreak;

    public Climber() {
        portWinch = new CANSparkMax(ClimberConstants.kPortWinchID, MotorType.kBrushless);
        starWinch = new CANSparkMax(ClimberConstants.kStarWinchID, MotorType.kBrushless);
        portWinch.setIdleMode(IdleMode.kBrake);
        starWinch.setIdleMode(IdleMode.kBrake);
        starWinch.setInverted(true);
        portEncoder = portWinch.getEncoder();
        starEncoder = starWinch.getEncoder();
        portController = portWinch.getPIDController();
        starController = starWinch.getPIDController();
        portController.setFeedbackDevice(portEncoder);
        portController.setP(ClimberConstants.kP);
        portController.setI(ClimberConstants.kI);
        portController.setD(ClimberConstants.kD);
        portController.setFF(ClimberConstants.kF);
        portController.setOutputRange(-.6, .6);
        starController.setFeedbackDevice(starEncoder);
        starController.setP(ClimberConstants.kP);
        starController.setI(ClimberConstants.kI);
        starController.setD(ClimberConstants.kD);
        starController.setFF(ClimberConstants.kF); 
        starController.setOutputRange(-.6, .6);
        portBeamBreak = new BeamBreak(ClimberConstants.kPortBeamBreakID);
        starBeamBreak = new BeamBreak(ClimberConstants.kStarBeamBreakID);
        portWinch.setSmartCurrentLimit(ClimberConstants.kClimberCurrentLimit);
        starWinch.setSmartCurrentLimit(ClimberConstants.kClimberCurrentLimit);
    }

    public void overridePosition() {
        portController.setReference(0, ControlType.kVoltage);
        starController.setReference(0, ControlType.kVoltage);
        portEncoder.setPosition(0);
        starEncoder.setPosition(0);
    }

    public void raise() {
        portController.setReference(ClimberConstants.kPortClimberTopPosition, ControlType.kPosition);
        starController.setReference(ClimberConstants.kStarClimberTopPosition, ControlType.kPosition);
    }

    public void lower() {
        portController.setReference(ClimberConstants.kPortClimberBottomPosition, ControlType.kPosition);
        starController.setReference(ClimberConstants.kStarClimberBottomPosition, ControlType.kPosition);
    }

    public void slowUp() {
        portController.setReference(portEncoder.getPosition() + 1, ControlType.kPosition);
        starController.setReference(starEncoder.getPosition() + 1, ControlType.kPosition);
    }

    public void slowDown() {
        portController.setReference(portEncoder.getPosition() - 1, ControlType.kPosition);
        starController.setReference(starEncoder.getPosition() - 1, ControlType.kPosition);
    }

    /**
     * 
     * @param up true is up, false is down
     */
    public void slowPort(boolean up) {
        if(up) {
            portController.setReference(portEncoder.getPosition() + 1, ControlType.kPosition);
        } else {
            portController.setReference(portEncoder.getPosition() - 1, ControlType.kPosition);
        }
    }

    /**
     * 
     * @param up true is up, false is down
     */
    public void slowStar(boolean up) {
        if(up) {
            starController.setReference(starEncoder.getPosition() + 1, ControlType.kPosition);
        } else {
            starController.setReference(starEncoder.getPosition() - 1, ControlType.kPosition);
        }
    }

    public void printVoltages() {
        System.out.println("PORT" + portWinch.getOutputCurrent());
        System.out.println("STAR" + starWinch.getOutputCurrent());
    }

    public void printPositions() {
        System.out.println("PORT" + portEncoder.getPosition());
        System.out.println("STAR" + starEncoder.getPosition()); 
    }

    public void beamBreakCheck() {
        if(portBeamBreak.isBroken()) {
            portWinch.set(0);
        }
        if(starBeamBreak.isBroken()) {
            starWinch.set(0);
        }
    }

    public void bringToZero() {
        if(portBeamBreak.isBroken()) {
            portController.setReference(0, ControlType.kVoltage);
            portWinch.set(0);
            portEncoder.setPosition(0);
        } else {
            portWinch.set(-.05);
        }
        if(starBeamBreak.isBroken()) {
            starController.setReference(0, ControlType.kVoltage);
            starWinch.set(0);
            starEncoder.setPosition(0);
        } else {
            starWinch.set(-.05);
        }
    }
}
