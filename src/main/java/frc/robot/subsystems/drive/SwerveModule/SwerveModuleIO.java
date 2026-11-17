package frc.robot.subsystems.drive.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface SwerveModuleIO extends Subsystem {
    public default void setVelocity(double velocity) {}

    public default void setRotation(Rotation2d rotation) {}

    public default void setState(SwerveModuleState state) {}

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
        return null;
    }

    public default SwerveModulePosition getPosition() {
        return null;
    }
}
