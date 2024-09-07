package org.firstinspires.ftc.teamcode.Algo;

import java.util.ArrayList;
import java.util.List;

public class TaskScheduler {

    private List<PostponedTask> tasks = new ArrayList<>();

    public TaskScheduler() {
    }

    public void schedule(final PostponedTask task) {
        task.reset();
        tasks.add(task);
    }

    public void update() {
        for(int i = 0; i < tasks.size(); ) {
            if(tasks.get(i).isDone()) {
                tasks.get(i).run();
                tasks.remove(i);
            }
            else i++;
        }
    }

}

