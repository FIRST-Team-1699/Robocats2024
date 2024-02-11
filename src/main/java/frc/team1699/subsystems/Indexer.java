package frc.team1699.subsystems;

import java.util.jar.Manifest;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.team1699.Constants.IndexerConstants;

public class Indexer {
    private IndexStates wantedState;
    private IndexStates currentState;
    private CANSparkMax indexMotor;
    private boolean hasNote;
    private Manifest manifesting;

    public Indexer() {
        indexMotor = new CANSparkMax(IndexerConstants.kMotorID, MotorType.kBrushless);
        hasNote = false;
        manifesting = new Manifest();
    }

    public boolean isLoaded() {
        // TODO check if there is a note present with a sensor
        return false;
    }

    public void update() {
        if(isLoaded()) {
            hasNote = true;
        } else {
            hasNote = false;
        }
        switch(currentState) {
            case EMPTY:
                break;
            case FEEDING:
                break;
            case LOADED:
                break;
            case REVERSING:
                break;
            case INTAKING:
                if(hasNote) {
                    setWantedState(IndexStates.LOADED);
                }
            default:
                break;

        }
    }

    private void handleStateTransition() {
        switch(wantedState) {
            case EMPTY:
                indexMotor.set(0);
                break;
            case FEEDING:
                indexMotor.set(IndexerConstants.kIndexerSpeed);
                break;
            case LOADED:
                indexMotor.set(0);
                break;
            case REVERSING:
                indexMotor.set(-IndexerConstants.kIndexerSpeed);
                break;
            case INTAKING:
                indexMotor.set(IndexerConstants.kIndexerSpeed);
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

    public IndexStates getCurrentState() {
        return currentState;
    }

    public enum IndexStates {
        LOADED,
        EMPTY,
        FEEDING,
        REVERSING,
        INTAKING
    }
}
