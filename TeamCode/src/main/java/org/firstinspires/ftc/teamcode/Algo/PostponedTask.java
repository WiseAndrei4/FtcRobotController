package org.firstinspires.ftc.teamcode.Algo;

import org.firstinspires.ftc.teamcode.Algo.Time.Timer;

public abstract class PostponedTask implements Task {

    private final Timer timer;
    
    public PostponedTask(final double delay) {
        timer = new Timer(delay);
    }

    public void reset() {
        timer.reset();
    }

    public boolean isDone() {
        return timer.isDone();
    }

}
