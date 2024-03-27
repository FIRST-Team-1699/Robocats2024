package frc.team1699.lib.auto.events;

import edu.wpi.first.wpilibj.Timer;
import frc.team1699.subsystems.Manipulator;
import frc.team1699.subsystems.Manipulator.ManipulatorStates;

public class SpeakerShootGoofyEvent extends Event {
    private Manipulator manipulator;
    private Timer timer;

    public SpeakerShootGoofyEvent(Manipulator manipulator) {
        this.manipulator = manipulator;
        this.timer = new Timer();
        timer.stop();
        timer.reset();
    }

    @Override
    public void initialize() {
        manipulator.setWantedState(ManipulatorStates.SPEAKER_GOOFY_SHOOT);
        timer.start();
    }

    @Override
    public void update() {
        if(timer.advanceIfElapsed(.5)) {
            manipulator.setWantedState(ManipulatorStates.SHOOTING);
        }
    }

    @Override
    public boolean isFinished() {
        return !manipulator.isLoaded();
    }

    @Override
    public void finish() {
        manipulator.setWantedState(ManipulatorStates.IDLE);
    }
}
