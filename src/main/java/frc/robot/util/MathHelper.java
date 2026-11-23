package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

public class MathHelper {
    public static Rotation2d minDist180(Rotation2d rot1, Rotation2d rot2) {
        double diff =
                Math.abs(
                        MathUtil.inputModulus(rot1.getRadians(), 0, Math.PI)
                                - MathUtil.inputModulus(rot2.getRadians(), 0, Math.PI));
        if (diff > Math.PI / 2) diff = Math.PI - diff;
        return new Rotation2d(diff);
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
}
