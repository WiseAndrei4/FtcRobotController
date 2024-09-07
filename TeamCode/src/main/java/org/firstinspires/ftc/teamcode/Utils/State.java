package org.firstinspires.ftc.teamcode.Utils;

public class State<T> {

    private T oldState, currentState;

    public State(T currentState) {
        this.oldState = currentState;
        this.currentState = currentState;
    }

    public void update(T state) {
        oldState = currentState;
        currentState = state;
    }

    public T getState() {
        return this.currentState;
    }

    public boolean justChanged() {
        return oldState != currentState;
    }

}
