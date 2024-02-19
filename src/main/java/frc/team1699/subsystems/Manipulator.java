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

    private PivotPoses lastPose;

    public Manipulator() {
        intake = new Intake();
        indexer = new Indexer();
        pivot = new Pivoter();
        shooter = new Shooter();
        this.lastPose = PivotPoses.IDLE;
    }

    public void startOrchestra() {
        shooter.startOrchestra();
    }

    public void update() {
        if(indexer.isLoaded()) {
            isLoaded = true;
        } else {
            isLoaded = false;
        }
        switch(currentState) {
            case SHOOTING:
                if(shooter.atSpeed() && pivot.isAtAngle()) {
                    indexer.setWantedState(IndexStates.FEEDING);
                }
                break;
            case IDLE:
                break;
            case INTAKING:
                if(isLoaded) {
                    indexer.setWantedState(IndexStates.LOADED);
                }
                break;
            case OUTTAKING:
                break;
            case STORED:
                break;
            case TRAP_SHOOT:
                break;
            case AMP_SHOOT:
                break;
            case SPEAKER_SUB_SHOOT:
                break;
            case SPEAKER_LL_SHOOT:
                pivot.setAngle(Vision.getInstance().getSpeakerAngle());
                break;
            default:
                break;
        }
        indexer.update();
        intake.update();
        shooter.update();
    }

    private void handleStateTransition() {
        switch(wantedState) {
            case SHOOTING:
                switch(lastPose) {
                    case AMP:
                        shooter.setSpeed(ManipulatorConstants.kAmpSpeed);
                        break;
                    case IDLE:
                        break;
                    case SPEAKER_SUB:
                        shooter.setSpeed(ManipulatorConstants.kSpeakerSubwooferSpeed);
                        break;
                    case SPEAKER_AUTO:
                        shooter.setSpeed(ManipulatorConstants.kSpeakerSubwooferSpeed);
                    default:
                        break;

                }
                // TODO tell the pivoter to start aiming
                break;
            case IDLE:
                intake.setWantedState(IntakeStates.IDLE);
                indexer.setWantedState(IndexStates.EMPTY);
                // pivot.setAngle(ManipulatorConstants.kIdleAngle);
                shooter.setSpeed(0.0);
                break;
            case INTAKING:
                // TODO check if you are loaded. if so, you can't intake and the transition is failed
                intake.setWantedState(IntakeStates.INTAKING);
                pivot.setAngle(ManipulatorConstants.kIntakeAngle);
                indexer.setWantedState(IndexStates.INTAKING);
                shooter.setSpeed(0);
                break;
            case OUTTAKING:
                intake.setWantedState(IntakeStates.REVERSING);
                pivot.setAngle(ManipulatorConstants.kIntakeAngle);
                shooter.setSpeed(0);
                indexer.setWantedState(IndexStates.REVERSING);
            case STORED:
                intake.setWantedState(IntakeStates.IDLE);
                pivot.setAngle(ManipulatorConstants.kIdleAngle);
                indexer.setWantedState(IndexStates.LOADED);
                break;
            case TRAP_SHOOT:
                // TODO check if you are loaded (with indexer.isLoaded()). if you aren't, the state transition is failed and you go back to idle
                pivot.setAngle(ManipulatorConstants.kTrapAngle);
                shooter.setSpeed(ManipulatorConstants.kTrapSpeed);
                break;
            case AMP_SHOOT:
                pivot.setAngle(ManipulatorConstants.kAmpAngle);
                shooter.setSpeed(ManipulatorConstants.kAmpSpeed);
                lastPose = PivotPoses.AMP;
                break;
            case SPEAKER_SUB_SHOOT:
                pivot.setAngle(ManipulatorConstants.kSpeakerSubwooferAngle);
                lastPose = PivotPoses.SPEAKER_SUB;
                break;
            case SPEAKER_LL_SHOOT:
                lastPose = PivotPoses.SPEAKER_AUTO;
                break;
            default:
                break;
            
        }
        currentState = wantedState; // I MIGHT TAKE THIS OUT LATER IF I WANT TO MAKE IT SO THAT SOMETIMES YOU DONT SUCCESSFULLY GO INTO A STATE
    }

    public void setWantedState(ManipulatorStates state) {
        if(this.wantedState != state) {
            wantedState = state;
        }
        handleStateTransition();
    }

    public ManipulatorStates getCurrentState() {
        return currentState;
    }

    public boolean shooterAtSpeed() {
        return shooter.atSpeed();
    }

    public boolean isLoaded() {
        return indexer.isLoaded();
    }

    public enum PivotPoses {
        SPEAKER_SUB,
        SPEAKER_AUTO,
        AMP,
        IDLE
    }

    public enum ManipulatorStates {
        INTAKING,
        OUTTAKING,
        SHOOTING,
        IDLE,
        STORED,
        SPEAKER_SUB_SHOOT,
        SPEAKER_LL_SHOOT,
        TRAP_SHOOT,
        AMP_SHOOT
    }
}