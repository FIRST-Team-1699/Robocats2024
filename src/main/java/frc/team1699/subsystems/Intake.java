package frc.team1699.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.team1699.Constants.IntakeConstants;

public class Intake {
    private CANSparkMax intakeMotor;
    private IntakeStates wantedState;
    private IntakeStates currentState = IntakeStates.IDLE;

    public Intake() {
        intakeMotor = new CANSparkMax(IntakeConstants.kMotorID, MotorType.kBrushless);
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
                intakeMotor.set(0);
                break;
            case INTAKING:
                intakeMotor.set(IntakeConstants.kIntakeSpeed);
                break;
            case REVERSING:
                intakeMotor.set(-IntakeConstants.kIntakeSpeed);
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
