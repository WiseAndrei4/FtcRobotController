package org.firstinspires.ftc.teamcode.State;

import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class State {

    public abstract State update();

    public void preInit() {}
    public void postInit() {}

    private final String name;

    private final ElapsedTime elapsedTime;
    private final double waitTime;

    private boolean isInitialized = false;

    public State() {
        this("UNKNOWN", 0.0);
    }

    public State(final String name) {
        this(name, 0.0);
    }

    public State(final String name, final double waitTime) {
        this.elapsedTime = new ElapsedTime();
        this.waitTime = waitTime;
        this.name = name;
    }

    public void internalInit() {
        isInitialized = false;
        this.elapsedTime.reset();
        preInit();
    }

    public State internalUpdate() {
        if(this.elapsedTime.time() >= this.waitTime) {
            if(!isInitialized) {
                isInitialized = true;
                postInit();
            }

            return update();
        }
        return null;
    }

    public String getName() {
        return this.name;
    }

}
