package org.firstinspires.ftc.bobot.limelight;

public class SampleDetection {

    private final double dx;
    private final double dy;
    private final boolean fresh;

    public SampleDetection(double dx, double dy, boolean fresh) {
        this.dx = dx;
        this.dy = dy;
        this.fresh = fresh;
    }

    public double getDx() {
        return dx;
    }

    public double getDy() {
        return dy;
    }

    public boolean isFresh() {
        return fresh;
    }
}
