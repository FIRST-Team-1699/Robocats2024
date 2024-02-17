package frc.team1699.subsystems;

import frc.team1699.Constants.ManipulatorConstants;
import frc.team1699.subsystems.Indexer.IndexStates;
import frc.team1699.subsystems.Intake.IntakeStates;

public class Manipulator {
    private Intake intake;
    private Indexer indexer;
    private Pivoter pivot;
    private Shooter shooter;

    private ManipulatorStates wantedState;
    private ManipulatorStates currentState;

    private boolean isLoaded;

    public Manipulator() {
        intake = new Intake();
        indexer = new Indexer();
        pivot = new Pivoter();
        shooter = new Shooter();
    }

    public void startOrchestra() {
        shooter.startOrchestra();
    }

    public void update() {
        switch(currentState) {
            case AIMING:
                break;
            case IDLE:
                break;
            case INTAKING:
                break;
            case STORED:
                break;
            case TRAP_SHOOT:
                if(shooter.atSpeed()) {
                    indexer.setWantedState(IndexStates.FEEDING);
                }
                break;
            case AMP_SHOOT:
                if(shooter.atSpeed()) {
                    indexer.setWantedState(IndexStates.FEEDING);
                }
                break;
            case SPEAKER_SHOOT:
                if(shooter.atSpeed()) {
                    indexer.setWantedState(IndexStates.FEEDING);
                }
                break;
            default:
                break;
        }
        indexer.update();
        intake.update();
        shooter.update();
    }

    private void handleStateTransition() {
        // TODO add indexer stuff after we know about indexer
        switch(wantedState) {
            case AIMING:
                // TODO tell the pivoter to start aiming
                break;
            case IDLE:
                intake.setWantedState(IntakeStates.IDLE);
                pivot.setAngle(ManipulatorConstants.kIdleAngle);
                break;
            case INTAKING:
                // TODO check if you are loaded. if so, you can't intake and the transition is failed
                intake.setWantedState(IntakeStates.INTAKING);
                pivot.setAngle(ManipulatorConstants.kIntakeAngle);
                shooter.setSpeed(0);
                break;
            case STORED:
                intake.setWantedState(IntakeStates.IDLE);
                pivot.setAngle(ManipulatorConstants.kIdleAngle);
                break;
            case TRAP_SHOOT:
                // TODO check if you are loaded (with indexer.isLoaded()). if you aren't, the state transition is failed and you go back to idle
                pivot.setAngle(ManipulatorConstants.kTrapAngle);
                shooter.setSpeed(ManipulatorConstants.kTrapSpeed);
                break;
            case AMP_SHOOT:
                pivot.setAngle(ManipulatorConstants.kAmpAngle);
                shooter.setSpeed(ManipulatorConstants.kAmpSpeed);
                break;
            case SPEAKER_SHOOT:
                pivot.setAngle(ManipulatorConstants.kSpeakerSubwooferAngle);
                shooter.setSpeed(ManipulatorConstants.kSpeakerSubwooferSpeed);
                break;
            default:
                break;
            
        }
        currentState = wantedState; // I MIGHT TAKE THIS OUT LATER IF I WANT TO MAKE IT SO THAT SOMETIMES YOU DONT SUCCESSFULLY GO INTO A STATE
    }

    public void setWantedState(ManipulatorStates state) {
        if(this.wantedState != state) {
            wantedState = state;
            handleStateTransition();
        }
    }

    public ManipulatorStates getCurrentState() {
        return currentState;
    }

    public boolean shooterAtSpeed() {
        return shooter.atSpeed();
    }

    public enum ManipulatorStates {
        INTAKING,
        SPEAKER_SHOOT,
        AIMING,
        IDLE,
        STORED,
        TRAP_SHOOT,
        AMP_SHOOT
    }
}