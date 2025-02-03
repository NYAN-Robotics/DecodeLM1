package org.firstinspires.ftc.teamcode.utilities.datastructures;

import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Pose;

import java.util.LinkedList;
import java.util.Queue;

public class CentripetalBuffer {

    public static final int DEFAULT_CAPACITY = 3;

    private final int theCapacity;
    private final Queue<Pose> theContainer1;
    private final Queue<Pose> theContainer2;
    private final Queue<Pose> theContainer3;

    public CentripetalBuffer(int aCapacity) {
        this.theCapacity = aCapacity;

        this.theContainer1 = new LinkedList<>();
        this.theContainer2 = new LinkedList<>();
        this.theContainer3 = new LinkedList<>();
    }

    public CentripetalBuffer() {
        this(DEFAULT_CAPACITY);
    }

    public void addPose(Pose pose) {
        if (theContainer1.size() < theCapacity) {
            theContainer1.add(pose);
        } else if (theContainer2.size() < theCapacity) {
            theContainer2.add(theContainer1.poll());
            theContainer1.add(pose);
        } else if (theContainer3.size() < theCapacity) {
            theContainer3.add(theContainer2.poll());
            theContainer2.add(theContainer1.poll());
            theContainer1.add(pose);
        } else {
            theContainer3.poll();
            theContainer3.add(theContainer2.poll());
            theContainer2.add(theContainer1.poll());
            theContainer1.add(pose);
        }
    }

    private Pose getAveragePose(Queue<Pose> aContainer) {
        Pose averagePose = new Pose();

        for (Pose pose : aContainer) {
            averagePose.add(pose);
        }

        averagePose.times(1.0 / aContainer.size());

        return averagePose;
    }


    public Pose[] getAveragePoses() {
        return new Pose[] {
                getAveragePose(theContainer1),
                getAveragePose(theContainer2),
                getAveragePose(theContainer3)
        };
    }
}