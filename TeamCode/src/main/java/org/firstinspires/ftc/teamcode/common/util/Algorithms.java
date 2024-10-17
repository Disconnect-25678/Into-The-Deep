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
        double[] vals = new double[4];
        for (int i = 0; i < vals.length; i++) {
            Vector2d v;
            if (i == 0 || i == 3) {
                v = driveVector.project(iHat);
                vals[i] = v.magnitude() * Math.signum(v.getX());
            } else {
                v = driveVector.project(jHat);
                vals[i] = v.magnitude() * Math.signum(v.getY());
            }
        }
        double s = Math.max(Math.abs(vals[0]), Math.abs(vals[1]));
        if (Double.compare(0, s) != 0) {
            for (int i = 0; i < vals.length; i++) {
                vals[i] = (vals[i] / s) * driveVector.magnitude();
            }
        }
        vals[0] += rotation;
        vals[1] -= rotation;
        vals[2] += rotation;
        vals[3] -= rotation;
        double max = 0;
        for (int i = 0; i < vals.length; i++) {
            vals[i] = vals[i] * scale;
            max = Math.max(max, Math.abs(vals[i]));
        }
        if (max > 1) {
            for (int i = 0; i < vals.length; i++) {
                vals[i] = vals[i] / max;
            }
        }
        return vals;
    }
}
