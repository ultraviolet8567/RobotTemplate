package frc.robot.subsystems.drive.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveModuleConstants;

public class SwerveModule extends SubsystemBase {
    private final SwerveModuleIO io;

    public SwerveModule(SwerveModuleIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        if (Math.abs(getAbsoluteRotation().minus(getRotation()).getRadians())
                > SwerveModuleConstants.kAllowedTurnError) {
            resetEncoder();
        }
    }

    public void setRotation(Rotation2d rotation) {
        io.setRotation(rotation);
    }

    public void setVelocity(double velocity) {
        io.setVelocity(velocity);
    }

    public void resetEncoder() {
        io.resetEncoder();
    }

    public void stop() {
        io.stop();
    }

    public void setState(SwerveModuleState state) {
        io.setState(state);
    }

    public double getVelocity() {
        return getState().speedMetersPerSecond;
    }

    public Rotation2d getRotation() {
        return getState().angle;
    }

    public Rotation2d getAbsoluteRotation() {
        return io.getAbsoluteRotation();
    }

    public SwerveModuleState getState() {
        return io.getState();
    }

    public SwerveModulePosition getPosition() {
        return io.getPosition();
    }
}
