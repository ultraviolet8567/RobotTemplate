package frc.robot.subsystems.drive.SwerveModule;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.Constants.SwerveModulePreset;
import frc.robot.util.MathHelper;

public class SwerveModuleIOSparkMax implements SwerveModuleIO {
    public final String name;
    private final SparkMax driveMotor, turnMotor;
    private final SparkMaxConfig driveConfig, turnConfig;
    private final AnalogInput absoluteEncoder;
    private final RelativeEncoder driveEncoder, turnEncoder;
    private final SparkClosedLoopController driveControl, turnControl;
    private final double encoderOffset;

    public SwerveModuleIOSparkMax(SwerveModulePreset preset) {
        name = preset.name;

        driveMotor = new SparkMax(preset.drivePort, MotorType.kBrushless);
        turnMotor = new SparkMax(preset.turnPort, MotorType.kBrushless);

        driveConfig = new SparkMaxConfig();
        turnConfig = new SparkMaxConfig();

        driveConfig.voltageCompensation(12.0);
        driveConfig.smartCurrentLimit(40);
        driveConfig.inverted(preset.driveInverted);
        driveConfig.idleMode(IdleMode.kBrake);
        driveConfig.closedLoop.pid(
                SwerveModuleConstants.kSparkMaxGains.driveP(),
                SwerveModuleConstants.kSparkMaxGains.driveI(),
                SwerveModuleConstants.kSparkMaxGains.driveD());
        driveConfig.closedLoop.feedForward.kV(SwerveModuleConstants.kSparkMaxGains.driveV());
        driveConfig.closedLoopRampRate(0.05);

        driveEncoder = driveMotor.getEncoder();
        driveConfig.encoder.positionConversionFactor(preset.driveRot2Meters());
        driveConfig.encoder.velocityConversionFactor(preset.driveRPM2MetersPerSec());

        turnConfig.voltageCompensation(12.0);
        turnConfig.smartCurrentLimit(40);
        turnConfig.inverted(preset.turnInverted);
        turnConfig.idleMode(IdleMode.kBrake);
        turnConfig.closedLoop.pid(
                SwerveModuleConstants.kSparkMaxGains.turnP(),
                SwerveModuleConstants.kSparkMaxGains.turnI(),
                SwerveModuleConstants.kSparkMaxGains.turnD());
        turnConfig.closedLoop.positionWrappingEnabled(true);
        turnConfig.closedLoop.positionWrappingInputRange(0, Math.PI * 2);
        turnConfig.closedLoop.allowedClosedLoopError(
                SwerveModuleConstants.kAllowedTurnError, ClosedLoopSlot.kSlot0);
        turnConfig.closedLoopRampRate(0.05);

        turnEncoder = turnMotor.getEncoder();
        turnConfig.encoder.positionConversionFactor(preset.turnRot2Rads());
        turnConfig.encoder.velocityConversionFactor(preset.turnRPM2RadsPerSec());

        absoluteEncoder = new AnalogInput(preset.encoderPort);
        encoderOffset = preset.encoderOffset;
        turnEncoder.setPosition(getAbsoluteRotation().getRadians());

        driveMotor.configure(
                driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        turnMotor.configure(
                turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        driveControl = driveMotor.getClosedLoopController();
        turnControl = turnMotor.getClosedLoopController();
    }

    public void setVelocity(double velocity) {
        driveControl.setSetpoint(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }

    public void setPosition(double position) {
        driveControl.setSetpoint(0, ControlType.kVelocity);
    }

    public void setRotation(Rotation2d rotation) {
        turnControl.setSetpoint(rotation.getRadians(), ControlType.kPosition);
    }

    public void setState(SwerveModuleState state) {
        if (Math.abs(driveEncoder.getVelocity() - state.speedMetersPerSecond)
                > SwerveModuleConstants.kAllowedDriveError) setVelocity(state.speedMetersPerSecond);
        if (Math.abs(MathHelper.minDist180(state.angle, getRotation()).getRadians())
                > SwerveModuleConstants.kAllowedTurnError) setRotation(state.angle);
        else turnMotor.set(0);
    }

    public void resetEncoder() {
        turnEncoder.setPosition(getAbsoluteRotation().getRadians());
    }

    public void stop() {
        turnMotor.set(0);
        driveMotor.set(0);
    }

    public String getName() {
        return name;
    }

    public double getVelocity() {
        return driveMotor.getEncoder().getVelocity();
    }

    public Rotation2d getAbsoluteRotation() {
        double angle = absoluteEncoder.getAverageVoltage() / RobotController.getVoltage5V();
        angle = Units.rotationsToRadians(angle);
        angle += encoderOffset;
        angle = MathUtil.inputModulus(angle, -Math.PI, Math.PI);
        return new Rotation2d(angle);
    }

    public Rotation2d getRotation() {
        return new Rotation2d(turnEncoder.getPosition());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), getRotation());
    }
}
