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
        starWinch.follow(portWinch);
    }

    public void update() {
        switch(currentState) {
            case DOWN:
                break;
            case EXTENDING:
                // TODO check periodically if it is extended all the way. if it is, go into up state
                break;
            case RETRACTING:
                // TODO same as above
                break;
            case UP:
                break;
            default:
                break;
            
        }
    }

    private void handleStateTransition() {
        switch(wantedState) {
            case DOWN:
                portWinch.set(0);
                break;
            case EXTENDING:
                portWinch.set(ClimberConstants.kClimberSpeed);
                break;
            case RETRACTING:
                portWinch.set(-ClimberConstants.kClimberSpeed);
                break;
            case UP:
                portWinch.set(0);
                break;
            default:
                break;
        }
        currentState = wantedState;
    }


    public void setWantedState(ClimbStates state) {
            if(this.wantedState != state) {
                wantedState = state;
                handleStateTransition();
            }
    }

    public enum ClimbStates {
        EXTENDING,
        RETRACTING,
        UP,
        DOWN
    }
}
