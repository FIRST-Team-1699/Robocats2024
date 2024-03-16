package frc.team1699.subsystems;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
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

    private InterpolatingDoubleTreeMap pivotMap;

    public Manipulator() {
        intake = new Intake();
        indexer = new Indexer();
        pivot = new Pivoter();
        shooter = new Shooter();
        this.lastPose = PivotPoses.IDLE;
        pivotMap = new InterpolatingDoubleTreeMap();
        // key: angle offset, value: pivot angle
        pivotMap.put(14.9, 59.0);
        pivotMap.put(14.5, 56.0);
        pivotMap.put(14.0, 54.0);
        pivotMap.put(7.0, 43.0);
        pivotMap.put(0.0, 40.0);
        pivotMap.put(-7.0, 33.0);
        pivotMap.put(-12.0, 27.0);
        pivotMap.put(-15.0, 27.0);
        this.currentState = ManipulatorStates.IDLE;
        this.wantedState = ManipulatorStates.IDLE;
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
                if(ManipulatorConstants.kUseShooterTable) {
                    if(Vision.getInstance().hasTargetInView()) {
                        pivot.setAngle(pivotMap.get(Vision.getInstance().getTY()));
                    }
                } else {
                    pivot.setAngle(Vision.getInstance().getSpeakerAngle());
                }
                break;
            case SPEAKER_GOOFY_SHOOT:
                break;
            default:
                break;
        }
        indexer.update();
        intake.update();
        shooter.update();
        // pivot.printCurrent();
    }

    private void handleStateTransition() {
        switch(wantedState) {
            case SHOOTING:
                switch(lastPose) {
                    case AMP:
                        shooter.setSeparateSpeeds(ManipulatorConstants.kAmpTopSpeed, ManipulatorConstants.kAmpBottomSpeed);
                        break;
                    case IDLE:
                        shooter.setSpeed(ManipulatorConstants.kIdleSpeed);
                        break;
                    case SPEAKER_SUB:
                        shooter.setSpeed(ManipulatorConstants.kSpeakerSubwooferSpeed);
                        break;
                    case SPEAKER_LL:
                        // if(DriverStation.isAutonomous()) {
                        //     shooter.setSpeed(ManipulatorConstants.kSpeakerLLSpeed - 5);
                        // } else {
                        //     shooter.setSpeed(ManipulatorConstants.kSpeakerLLSpeed);
                        // }
                        shooter.setSpeed(-Vision.getInstance().getTY() + 38);
                        break;
                    case TRAP:
                        shooter.setSeparateSpeeds(ManipulatorConstants.kTopTrapSpeed, ManipulatorConstants.kBottomTrapSpeed);
                        break;
                    case SPEAKER_GOOFY_SHOOT:
                        shooter.setSpeed(ManipulatorConstants.kSpeakerLLSpeed);
                        break;
                    case SHUFFLE:
                        shooter.setSpeed(ManipulatorConstants.kShuffleAngle);
                    default:
                        break;

                }
                break;
            case IDLE:
                intake.setWantedState(IntakeStates.IDLE);
                indexer.setWantedState(IndexStates.EMPTY);
                shooter.setSpeed(ManipulatorConstants.kIdleSpeed);
                break;
            case INTAKING:
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
                break;
            case STORED:
                intake.setWantedState(IntakeStates.IDLE);
                pivot.setAngle(ManipulatorConstants.kIdleAngle);
                indexer.setWantedState(IndexStates.LOADED);
                shooter.setSpeed(0);
                break;
            case TRAP_SHOOT:
                pivot.setAngle(ManipulatorConstants.kTrapAngle);
                lastPose = PivotPoses.TRAP;
                break;
            case AMP_SHOOT:
                pivot.setAngle(ManipulatorConstants.kAmpAngle);
                shooter.setSeparateSpeeds(ManipulatorConstants.kAmpTopSpeed, ManipulatorConstants.kAmpBottomSpeed);
                lastPose = PivotPoses.AMP;
                break;
            case SHUFFLE:
                pivot.setAngle(ManipulatorConstants.kShuffleAngle);
                shooter.setSpeed(ManipulatorConstants.kShuffleAngle);
                lastPose = PivotPoses.SHUFFLE;
            case SPEAKER_SUB_SHOOT:
                pivot.setAngle(ManipulatorConstants.kSpeakerSubwooferAngle);
                lastPose = PivotPoses.SPEAKER_SUB;
                shooter.setSpeed(ManipulatorConstants.kSpeakerSubwooferSpeed);
                break;
            case SPEAKER_LL_SHOOT:
                lastPose = PivotPoses.SPEAKER_LL;
                shooter.setSpeed(ManipulatorConstants.kSpeakerLLSpeed);
                break;
            case SPEAKER_GOOFY_SHOOT:
                lastPose = PivotPoses.SPEAKER_GOOFY_SHOOT;
                pivot.setAngle(ManipulatorConstants.kGoofyAngle);
                shooter.setSpeed(ManipulatorConstants.kSpeakerLLSpeed);
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
        if(getCurrentState() == ManipulatorStates.SPEAKER_LL_SHOOT || getCurrentState() == ManipulatorStates.TRAP_SHOOT || getCurrentState() == ManipulatorStates.SPEAKER_SUB_SHOOT || getCurrentState() == ManipulatorStates.TRAP_SHOOT || getCurrentState() == ManipulatorStates.AMP_SHOOT) {
            return shooter.atSpeed();
        }
        return false;
    }

    public boolean pivotAtPose() {
        return pivot.isAtAngle();
    }

    public boolean isLoaded() {
        return indexer.isLoaded();
    }

    public enum PivotPoses {
        SPEAKER_SUB,
        SPEAKER_LL,
        AMP,
        TRAP,
        SPEAKER_GOOFY_SHOOT,
        SHUFFLE,
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
        AMP_SHOOT,
        SHUFFLE,
        SPEAKER_GOOFY_SHOOT
    }
}