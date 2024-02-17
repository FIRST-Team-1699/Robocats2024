package frc.team1699.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.team1699.Constants.IndexerConstants;
import frc.team1699.lib.sensors.BeamBreak;

public class Indexer {
    private IndexStates wantedState;
    private IndexStates currentState;
    private CANSparkMax indexMotor;
    private BeamBreak indexBeamBreak;
    private boolean hasNote;

    public Indexer() {
        indexMotor = new CANSparkMax(IndexerConstants.kMotorID, MotorType.kBrushless);
        indexBeamBreak = new BeamBreak(IndexerConstants.kBeamBreakID);
        hasNote = false;
    }

    public boolean isLoaded() {
        return indexBeamBreak.isBroken();
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
                if(!hasNote) {
                    setWantedState(IndexStates.EMPTY);
                }
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
