package frc.robot.subsystems.drive.Odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public interface OdometryIO {
    public default String getName() {
        return null;
    }

    public default void update() {}

    public default void setHeading(Rotation2d heading) {}

    public default void resetHeading() {}

    public default void addVisionPose(Pose2d pose, double timestamp) {}

    public default void setPose(Pose2d pose) {}

    public default Rotation2d getHeading() {
        return null;
    }

    public default Pose2d getPose() {
        return null;
    }

    public default Translation2d getAcceleration() {
        return null;
    }
}
