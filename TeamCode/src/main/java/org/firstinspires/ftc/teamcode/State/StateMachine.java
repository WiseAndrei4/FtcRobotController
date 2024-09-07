package org.firstinspires.ftc.teamcode.State;

public class StateMachine {

    private State currentState;

    public StateMachine() {
        this.currentState = null;
    }

    public void setState(State state) {
        this.currentState = state;
        this.currentState.internalInit();
    }

    public State getState() {
        return this.currentState;
    }

    public void update() {
        State next = currentState.internalUpdate();
        if(next != null) {
            currentState = next;
            currentState.internalInit();
        }
    }

}
