package frc.robot.subsystems.drive.Odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Odometry extends SubsystemBase {
    private final OdometryIO io;

    public Odometry(OdometryIO io) {
        System.out.println("[Init] Creating " + io.getName());
        this.io = io;
    }

    @Override
    public void periodic() {
        Logger.recordOutput(getName() + "/Pose", getHeading());
        io.update();
    }

    public void setHeading(Rotation2d heading) {
        io.setHeading(heading);
    }

    public void resetHeading() {
        io.resetHeading();
    }

    public void addVisionPose(Pose2d pose, double timestamp) {
        io.addVisionPose(pose, timestamp);
    }

    public void setPose(Pose2d pose) {
        io.setPose(pose);
    }

    public Rotation2d getHeading() {
        return io.getHeading();
    }

    public Pose2d getPose() {
        return io.getPose();
    }

    public Translation2d getAcceleration() {
        return io.getAcceleration();
    }
}
