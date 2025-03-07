package org.firstinspires.ftc.teamcode.common.util;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Point;

public class PublicPose {
    public double x;
    public double y;
    public double thetaDegrees;

    public PublicPose(double x, double y, double thetaDegrees) {
        this.setX(x);
        this.setY(y);
        this.setThetaDegrees(thetaDegrees);
    }

    public double getX() {
        return x;
    }

    public void setX(double x) {
        this.x = x;
    }

    public double getY() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
    }

    public double getThetaDegrees() {
        return thetaDegrees;
    }

    public void setThetaDegrees(double thetaDegrees) {
        this.thetaDegrees = thetaDegrees;
    }

    public double getThetaRadians() {
        return Math.toRadians(this.thetaDegrees);
    }

    public Pose toPedroPose() {
        return new Pose(
                this.getX(),
                this.getY(),
                this.getThetaRadians()
        );
    }

    public Point toPedroPoint() {
        return new Point(
                this.getX(),
                this.getY(),
                Point.CARTESIAN
        );
    }
}
