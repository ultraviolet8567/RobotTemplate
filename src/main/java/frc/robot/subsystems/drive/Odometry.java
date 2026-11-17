package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DriveConstants;
import org.littletonrobotics.junction.Logger;

public class Odometry extends SubsystemBase {
    private final Swerve swerve;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Pigeon2 pigeon;

    public Odometry(Swerve swerve) {
        System.out.println("[Init] Creating Odometry");

        this.swerve = swerve;

        this.pigeon = new Pigeon2(CAN.kPigeonPort);
        this.poseEstimator =
                new SwerveDrivePoseEstimator(
                        DriveConstants.kDriveKinematics,
                        pigeon.getRotation2d(),
                        swerve.getPositions(),
                        new Pose2d());
    }

    @Override
    public void periodic() {
        Logger.recordOutput(getName() + "/Pose", getHeading());
        poseEstimator.update(getHeading(), swerve.getPositions());
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
