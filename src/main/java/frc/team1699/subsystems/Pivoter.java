package frc.team1699.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.team1699.Constants.PivoterConstants;

public class Pivoter {
    public CANSparkMax pivotMotor;
    private pivoterStates wantedState;
    private pivoterStates currentState;

    public Pivoter() {
        pivotMotor = new CANSparkMax(PivoterConstants.kMotorID, MotorType.kBrushless);
    }

    public void update() { // TODO maybe use some real time tracking using vision
        switch (currentState) {
            case AMP:
                break;
            case SPEAKER:
                break;
            case STORED:
                break;
            case TRAP:
                break;
            default:
                break;
        }
    }
    private void handleStateTransition() { // TODO set the pivoter to the respective position initially
        switch(wantedState) {
            case AMP:
                break;
            case SPEAKER:
                break;
            case STORED:
                break;
            case TRAP:
                break;
            default:
                break;
            
        }
    }

    public void setWantedState(pivoterStates state) {
            if(this.wantedState != state) {
                wantedState = state;
                handleStateTransition();
            }
    }
    public enum pivoterStates {
        SPEAKER,
        AMP,
        TRAP,
        STORED
    }
}
