package frc.robot.subsystems.drive.Odometry;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Swerve;

public class OdometryIOPidgeon implements OdometryIO {
    private final Swerve swerve;
    private final Pigeon2 pigeon;
    private final SwerveDrivePoseEstimator poseEstimator;

    public OdometryIOPidgeon(Swerve swerve) {
        this.swerve = swerve;
        this.pigeon = new Pigeon2(CAN.kPigeonPort);

        this.poseEstimator =
                new SwerveDrivePoseEstimator(
                        DriveConstants.kDriveKinematics,
                        pigeon.getRotation2d(),
                        swerve.getPositions(),
                        new Pose2d());
    }

    public void update() {
        poseEstimator.update(getHeading(), swerve.getPositions());
    }

    public String getName() {
        return "Odometry";
    }

    public void setHeading(Rotation2d heading) {
        pigeon.setYaw(heading.getDegrees());
    }

    public void resetHeading() {
        pigeon.setYaw(0);
    }

    public void addVisionPose(Pose2d pose, double timestamp) {
        poseEstimator.addVisionMeasurement(pose, timestamp);
    }

    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(pigeon.getRotation2d(), swerve.getPositions(), pose);
    }

    public Rotation2d getHeading() {
        return pigeon.getRotation2d();
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }
}
