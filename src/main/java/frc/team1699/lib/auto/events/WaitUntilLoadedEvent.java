package frc.team1699.lib.auto.events;

import edu.wpi.first.wpilibj.Timer;
import frc.team1699.subsystems.Manipulator;

public class WaitUntilLoadedEvent extends Event {
    private Manipulator manipulator;
    private boolean intakeBreak;
    private Timer timer;

    public WaitUntilLoadedEvent(Manipulator manipulator) {
        this.manipulator = manipulator;
        this.intakeBreak = false;
        this.timer = new Timer();
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void update() {
        if(manipulator.intakeLoaded()) {
            intakeBreak = true;
        }
    }

    @Override
    public boolean isFinished() {
        if(manipulator.isLoaded() || (!intakeBreak && timer.get() > 5.0)) {
            return true;
        }
        return false;
    }

    @Override
    public void finish() {}
}