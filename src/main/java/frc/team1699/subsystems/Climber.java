package frc.team1699.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.team1699.Constants.ClimberConstants;;

public class Climber {
    private CANSparkMax portWinch, starWinch;
    private SparkPIDController portController, starController;
    private RelativeEncoder portEncoder, starEncoder;

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
    }

    public void overridePosition() {
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
}
