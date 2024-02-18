package frc.team1699.lib.auto.events;

import edu.wpi.first.wpilibj.Timer;
import frc.team1699.subsystems.Manipulator;

public class WaitUntilLoadedEvent extends Event {
    private Timer waitTimer;
    private Manipulator manipulator;

    public WaitUntilLoadedEvent(Manipulator manipulator) {
        waitTimer = new Timer();
        this.manipulator = manipulator;
    }

    @Override
    public void initialize() {
        waitTimer.start();
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        if(manipulator.loaded()) {
            return true;
        }
        return false;
    }

    @Override
    public void finish() {
        waitTimer.stop();
    }
}