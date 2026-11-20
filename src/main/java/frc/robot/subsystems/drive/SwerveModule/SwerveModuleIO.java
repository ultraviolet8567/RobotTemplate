package frc.robot.subsystems.drive.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModuleIO {
    public default String getName() {
        return null;
    }

    public default void setVelocity(double velocity) {}

    public default void setPosition(double position) {}

    public default void setRotation(Rotation2d rotation) {}

    public default void setState(SwerveModuleState state) {
        state.optimize(getRotation());
        setVelocity(state.speedMetersPerSecond);
        setRotation(state.angle);
    }

    public default void resetEncoder() {}

    public default void stop() {}

    public default double getVelocity() {
        return 0;
    }

    public default Rotation2d getAbsoluteRotation() {
        return null;
    }

    public default Rotation2d getRotation() {
        return null;
    }

    public default SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), getRotation());
    }

    public default SwerveModulePosition getPosition() {
        return null;
    }
}
