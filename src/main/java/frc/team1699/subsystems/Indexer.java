package frc.team1699.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.team1699.Constants.IndexerConstants;

public class Indexer {
    private IndexStates wantedState;
    private IndexStates currentState;
    public CANSparkMax indexMotor;
    public Indexer() {
        indexMotor = new CANSparkMax(IndexerConstants.kMotorID, MotorType.kBrushless);
    }

    public void update() {
        switch(currentState) {
            case EMPTY:
                // TODO check if you're loaded continually
                break;
            case FEEDING:
                break;
            case LOADED:
                break;
            case REVERSING:
                break;
            default:
                break;

        }
    }

    private void handleStateTransition() {
        switch(wantedState) {
            case EMPTY:
                break;
            case FEEDING:
                // TODO start feeding
                break;
            case LOADED:
                // TODO i feel like something needs to go here
                break;
            case REVERSING:
                // TODO start reversing
                break;
            default:
                break;
            
        }
        currentState = wantedState;
    }

    public void setWantedState(IndexStates state) {
            if(this.wantedState != state) {
                wantedState = state;
                handleStateTransition();
            }
    }

    public enum IndexStates {
        LOADED,
        EMPTY,
        FEEDING,
        REVERSING
    }
}
