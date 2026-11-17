package frc.robot.subsystems.drive.SwerveModule;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.MotorPreset;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.Constants.SwerveModulePreset;

public class SwerveModuleIOSim implements SwerveModuleIO {
    private final DCMotorSim driveMotor, turnMotor;
    private final PIDController driveControl, turnControl;

    public SwerveModuleIOSim(
            SwerveModulePreset preset, MotorPreset driveMotor, MotorPreset turnMotor) {

        this.driveMotor =
                new DCMotorSim(driveMotor.getLinearSystem(preset.driveRatio), driveMotor.motor);
        this.turnMotor =
                new DCMotorSim(
                        turnMotor.getLinearSystem(SwerveModuleConstants.kTurnGearRatio),
                        turnMotor.motor);

        turnControl =
                new PIDController(
                        SwerveModuleConstants.kSimPID.turnP(),
                        SwerveModuleConstants.kSimPID.turnI(),
                        SwerveModuleConstants.kSimPID.turnD());
        driveControl =
                new PIDController(
                        SwerveModuleConstants.kSimPID.driveP(),
                        SwerveModuleConstants.kSimPID.driveI(),
                        SwerveModuleConstants.kSimPID.driveD());
    }

    public void setVelocity(double velocity) {}

    public void setRotation(Rotation2d rotation) {}

    public void stop() {}

    public double getVelocity() {
        return 0;
    }

    public Rotation2d getRotation() {
        return null;
    }

    public SwerveModuleState getState() {
        return null;
    }

    public SwerveModulePosition getPosition() {
        return null;
    }
}
