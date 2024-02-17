package frc.team1699.lib.auto.events;

import edu.wpi.first.wpilibj.Timer;
import frc.team1699.subsystems.Manipulator;
import frc.team1699.subsystems.Manipulator.ManipulatorStates;

// TODO: make it not jank by having shooter logic to see if shooting is done, preferably not with a timer. Color sensor?
public class SpeakerShootEvent extends Event {
    private Manipulator manipulator;
    private boolean shooting;
    private Timer timer;

    public SpeakerShootEvent(Manipulator manipulator) {
        this.manipulator = manipulator;
        this.shooting = false;
        this.timer = new Timer();
        timer.stop();
        timer.reset();
    }

    @Override
    public void initialize() {
        manipulator.setWantedState(ManipulatorStates.SPEAKER_SHOOT);
    }

    @Override
    public void update() {
        if(manipulator.shooterAtSpeed() && !shooting) {
            timer.start();
            shooting = true;
        }
    }

    @Override
    public boolean isFinished() {
        if(timer.advanceIfElapsed(1.0)) {
            return true;
        }
        return false;
    }

    @Override
    public void finish() {
        manipulator.setWantedState(ManipulatorStates.IDLE);
    }
}
