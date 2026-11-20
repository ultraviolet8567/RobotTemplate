package frc.robot.subsystems.drive.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.util.MathHelper;
import org.littletonrobotics.junction.Logger;

public class SwerveModule extends SubsystemBase {
    private final SwerveModuleIO io;

    private double targetPosition = 0;

    public SwerveModule(SwerveModuleIO io) {
        System.out.println("[Init] Creating " + io.getName());
        this.io = io;
        updatePosition();
    }

    @Override
    public void periodic() {
        if (Math.abs(MathHelper.minDist180(getRotation(), getAbsoluteRotation()).getRadians())
                > SwerveModuleConstants.kAllowedTurnError) {
            resetEncoder();
        }

        Logger.recordOutput("Swerve/Modules/" + io.getName() + "/TargetPosition", targetPosition);
        Logger.recordOutput(
                "Swerve/Modules/" + io.getName() + "/CurrentPosition",
                getPosition().distanceMeters);
    }

    public void setRotation(Rotation2d rotation) {
        io.setRotation(rotation);
    }

    public void setVelocity(double velocity) {
        io.setVelocity(velocity);
    }

    public void updatePosition() {
        targetPosition = io.getPosition().distanceMeters;
    }

    public void holdPosition() {
        io.setPosition(targetPosition);
    }

    public void resetEncoder() {
        io.resetEncoder();
    }

    public void stop() {
        io.stop();
    }

    public void setState(SwerveModuleState state) {
        state.optimize(getRotation());
        Logger.recordOutput(
                "Swerve/Modules/" + io.getName() + "/Error",
                Math.abs(MathHelper.minDist180(state.angle, getRotation()).getRadians()));
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
