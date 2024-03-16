package frc.team1699.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.team1699.Constants.IntakeConstants;

public class Intake {
    private CANSparkMax deckIntakeMotor;
    private CANSparkMax bilgeIntakeMotor;
    private IntakeStates wantedState;
    private IntakeStates currentState = IntakeStates.IDLE;

    public Intake() {
        deckIntakeMotor = new CANSparkMax(IntakeConstants.kDeckMotorID, MotorType.kBrushless);
        bilgeIntakeMotor = new CANSparkMax(IntakeConstants.kBilgeMotorID, MotorType.kBrushless);
        bilgeIntakeMotor.setInverted(false);
    }

    public void update() {
        switch(currentState) {
            case IDLE:
                break;
            case INTAKING:
                break;
            case REVERSING:
                break;
            default:
                break;
        }
    }

    private void handleStateTransition() {
        switch(wantedState) {
            case IDLE:
                deckIntakeMotor.set(0);
                bilgeIntakeMotor.set(0);
                break;
            case INTAKING:
                deckIntakeMotor.set(IntakeConstants.kDeckSpeed);
                bilgeIntakeMotor.set(IntakeConstants.kBilgeSpeed);
                break;
            case REVERSING:
                deckIntakeMotor.set(-IntakeConstants.kDeckSpeed);
                bilgeIntakeMotor.set(-IntakeConstants.kBilgeSpeed);
                break;
            default:
                break;
        }
        currentState = wantedState;
    }

    public void setWantedState(IntakeStates state) {
        if(this.wantedState != state) {
            wantedState = state;
            handleStateTransition();
        }
    }

    public enum IntakeStates {
        INTAKING,
        IDLE,
        REVERSING
    }
}
