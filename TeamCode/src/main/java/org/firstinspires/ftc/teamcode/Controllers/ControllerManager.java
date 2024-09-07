package org.firstinspires.ftc.teamcode.Controllers;

import androidx.annotation.NonNull;

import java.util.HashMap;
import java.util.Map;

public class ControllerManager {

    // the controllers
    private final Map<String, Controller> controllers = new HashMap<>();

    // create a controller manager
    public ControllerManager() {
    }

    // add a controller to the manager; return the manager to chain calls
    public ControllerManager addController(@NonNull String name, @NonNull Controller controller) {
        controllers.put(name, controller);
        return this;
    }

    // return the controller with the given name and the given type
    public <T extends Controller> T get(@NonNull Class<T> cls, @NonNull String name) {
        // if the map has the controller
        if(controllers.containsKey(name)) {
            // get the controller
            Controller controller = controllers.get(name);

            // if same class
            if(cls.isInstance(controller)) {
                return (T) controller;
            }
        }

        // return nothing if not found
        return null;
    }

}
