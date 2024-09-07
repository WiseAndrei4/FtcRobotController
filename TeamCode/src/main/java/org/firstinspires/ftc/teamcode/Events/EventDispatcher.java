package org.firstinspires.ftc.teamcode.Events;

import java.util.ArrayList;
import java.util.List;

public class EventDispatcher<T> {

    private List<EventHandler<T>> handlers = new ArrayList<>();
    private List<T> eventQueue = new ArrayList<>();

    public void register(EventHandler<T> handler) {
        handlers.add(handler);
    }

    public void dispatch(T event) {
        for(EventHandler<T> handler : handlers) {
            handler.handle(event);
        }
    }

    public void dispatchAsync(T event) {
        eventQueue.add(event);
    }

    public void update() {
        for(T event : eventQueue) {
            for(EventHandler<T> handler : handlers) {
                handler.handle(event);
            }
        }

        eventQueue.clear();
    }
}
