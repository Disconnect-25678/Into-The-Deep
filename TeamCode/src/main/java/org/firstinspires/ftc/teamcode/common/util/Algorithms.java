package org.firstinspires.ftc.teamcode.common.util;

import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

public final class Algorithms {
    private Algorithms(){}

    public static Vector2d mapJoystick(double x, double y) {
        double m_x = x * Math.sqrt(1 - y * y / 2);
        double m_y = y * Math.sqrt(1 - x * x / 2);
        return new Vector2d(m_x * Math.sqrt(2), m_y * Math.sqrt(2));
    }

    public static double[] returnMecanumValues(Vector2d driveVector, double rotation) {
        return returnMecanumValues(driveVector, rotation, 1);
    }

    public static double[] returnMecanumValues(Vector2d driveVector, double rotation, double scale) {
        Vector2d iHat = new Vector2d(Math.cos(Math.PI/4), Math.sin(Math.PI / 4)); //FL and BR
        Vector2d jHat = new Vector2d(-Math.cos(Math.PI/4), Math.sin(Math.PI / 4)); //BL and FR
        double fl = driveVector.project(iHat).magnitude();
        double fr = driveVector.project(jHat).magnitude();
        double bl = driveVector.project(jHat).magnitude();
        double br = driveVector.project(iHat).magnitude();
        fl += rotation;
        fr -= rotation;
        bl += rotation;
        br -= rotation;
        double maxPower = Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br)));
        if (maxPower > 1) {
            fl /= maxPower;
            fr /= maxPower;
            bl = maxPower;
            br = maxPower;
        }
        fl *= scale;
        fr *= scale;
        bl *= scale;
        br *= scale;
        return new double[] {fl, fr, bl, br};
    }
}
