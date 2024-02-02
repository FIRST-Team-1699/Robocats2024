package frc.team1699.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.team1699.Constants.ClimberConstants;;

public class Climber {
    private ClimbStates currentState = ClimbStates.DOWN; 
    private ClimbStates wantedState = ClimbStates.DOWN;

    private CANSparkMax portWinch;
    private CANSparkMax starWinch;

    public Climber() {
        portWinch = new CANSparkMax(ClimberConstants.kPortWinchID, MotorType.kBrushless);
        starWinch = new CANSparkMax(ClimberConstants.kStarWinchID, MotorType.kBrushless);
    }

    public void update() {
        
    }

    public enum ClimbStates {
        EXTENDING,
        RETRACTING,
        UP,
        DOWN
    }
}
