package frc.robot.subsystems.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.subsystems.drive.SwerveModule.SwerveModule;
import frc.robot.subsystems.drive.SwerveModule.SwerveModuleIO;
import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {
    private final SwerveModule moduleFL, moduleBL, moduleFR, moduleBR;

    public Swerve(
            SwerveModuleIO moduleFL,
            SwerveModuleIO moduleFR,
            SwerveModuleIO moduleBL,
            SwerveModuleIO moduleBR) {
        System.out.println("[Init] Creating Swerve");

        this.moduleFL = new SwerveModule(moduleFL);
        this.moduleBL = new SwerveModule(moduleBL);
        this.moduleFR = new SwerveModule(moduleFR);
        this.moduleBR = new SwerveModule(moduleBR);
    }

    @Override
    public void periodic() {
        Logger.recordOutput(getName() + "/CurrentState", getStates());
    }

    public void setStates(SwerveModuleState[] moduleStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveModuleConstants.kMaxSpeed);
        moduleFL.setState(moduleStates[0]);
        moduleFR.setState(moduleStates[1]);
        moduleBL.setState(moduleStates[2]);
        moduleBR.setState(moduleStates[3]);

        Logger.recordOutput(getName() + "/TargetState", moduleStates);
    }

    public void setStates(ChassisSpeeds chassisSpeed) {
        SwerveModuleState[] moduleStates =
                DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeed);
        setStates(moduleStates);
    }

    public void updatePositions() {
        moduleFL.updatePosition();
        moduleFR.updatePosition();
        moduleBL.updatePosition();
        moduleBR.updatePosition();
    }

    public void holdPosition() {
        moduleFL.holdPosition();
        moduleFR.holdPosition();
        moduleBL.holdPosition();
        moduleBR.holdPosition();
    }

    public void lockWheels() {
        setStates(DriveConstants.kLockStates);
    }

    public void resetEncoders() {
        moduleFL.resetEncoder();
        moduleFR.resetEncoder();
        moduleBL.resetEncoder();
        moduleBR.resetEncoder();
    }

    public void stop() {
        moduleFL.stop();
        moduleFR.stop();
        moduleBL.stop();
        moduleBR.stop();
    }

    public SwerveModulePosition[] getPositions() {
        return new SwerveModulePosition[] {
            moduleFL.getPosition(),
            moduleBL.getPosition(),
            moduleFL.getPosition(),
            moduleFL.getPosition()
        };
    }

    public SwerveModuleState[] getStates() {
        return new SwerveModuleState[] {
            moduleFL.getState(), moduleFR.getState(), moduleBL.getState(), moduleBR.getState()
        };
    }

    public ChassisSpeeds getChassisSpeed() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(getStates());
    }
}
