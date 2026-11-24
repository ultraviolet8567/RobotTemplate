package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

public class MathHelper {
    public static Rotation2d minDist180(Rotation2d rot1, Rotation2d rot2) {
        double diff =
                MathUtil.inputModulus(rot1.getRadians(), -Math.PI / 2, Math.PI / 2)
                        - MathUtil.inputModulus(rot2.getRadians(), -Math.PI / 2, Math.PI / 2);
        if (diff > Math.PI / 2) diff = -Math.PI + diff;
        if (diff < -Math.PI / 2) diff = Math.PI - diff;
        return new Rotation2d(MathUtil.clamp(diff, -Math.PI / 2, Math.PI / 2));
    }

    public static Rotation2d minDist360(Rotation2d rot1, Rotation2d rot2) {
        double diff =
                MathUtil.inputModulus(rot1.getRadians(), -Math.PI, Math.PI)
                        - MathUtil.inputModulus(rot2.getRadians(), -Math.PI, Math.PI);
        if (diff > Math.PI) diff = -2 * Math.PI + diff;
        if (diff < -Math.PI) diff = 2 * Math.PI - diff;
        return new Rotation2d(MathUtil.clamp(diff, -Math.PI, Math.PI));
    }

    public static double moveToward(double from, double to, double speed) {
        if (Math.abs(to - from) < speed) {
            return to;
        } else if (from < to) {
            return from + speed;
        } else if (from > to) {
            return from - speed;
        }
        return from;
    }

    public static double length(double x, double y) {
        return Math.sqrt(x * x + y * y);
    }
}
