package frc.team1699.lib.auto.events;

import frc.team1699.subsystems.Manipulator;
import frc.team1699.subsystems.Manipulator.ManipulatorStates;

public class SpeakerShootLLEvent extends Event {
    private Manipulator manipulator;

    public SpeakerShootLLEvent(Manipulator manipulator) {
        this.manipulator = manipulator;
    }

    @Override
    public void initialize() {
        manipulator.setWantedState(ManipulatorStates.SPEAKER_LL_SHOOT);
    }

    @Override
    public void update() {
        if(manipulator.pivotAtPose()) {
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
