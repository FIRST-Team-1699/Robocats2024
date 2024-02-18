package frc.team1699.lib.auto.events;

import edu.wpi.first.wpilibj.Timer;
import frc.team1699.subsystems.Manipulator;
import frc.team1699.subsystems.Manipulator.ManipulatorStates;

// TODO: make it not jank by having shooter logic to see if shooting is done, preferably not with a timer. Color sensor?
public class SpeakerShootEvent extends Event {
    private Manipulator manipulator;
    private Timer timer;

    public SpeakerShootEvent(Manipulator manipulator) {
        this.manipulator = manipulator;
        this.timer = new Timer();
        timer.stop();
        timer.reset();
    }

    @Override
    public void initialize() {
        manipulator.setWantedState(ManipulatorStates.SPEAKER_SUB_SHOOT);
        manipulator.setWantedState(ManipulatorStates.SHOOTING);
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return manipulator.autoHasShot();
    }

    @Override
    public void finish() {
        manipulator.setWantedState(ManipulatorStates.IDLE);
    }
}
