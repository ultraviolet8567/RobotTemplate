package frc.robot.subsystems.drive.SwerveModule;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.Constants.SwerveModulePreset;

public class SwerveModuleIOTalonFX implements SwerveModuleIO {
    private final TalonFX driveMotor, turnMotor;
    private final TalonFXConfiguration driveConfig, turnConfig;
    private final Slot0Configs driveControlConfig, turnControlConfig;
    private final AnalogInput absoluteEncoder;
    private final double encoderOffset;
    private final SwerveModulePreset preset;

    public SwerveModuleIOTalonFX(SwerveModulePreset preset) {
        this.preset = preset;

        driveMotor = new TalonFX(preset.drivePort);
        turnMotor = new TalonFX(preset.turnPort);

        driveConfig = new TalonFXConfiguration();
        turnConfig = new TalonFXConfiguration();

        driveControlConfig = new Slot0Configs();
        turnControlConfig = new Slot0Configs();

        driveConfig.withCurrentLimits(
                new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(120)
                        .withStatorCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(40)
                        .withSupplyCurrentLimitEnable(true));
        driveConfig.withMotorOutput(
                new MotorOutputConfigs()
                        .withInverted(
                                preset.driveInverted
                                        ? InvertedValue.CounterClockwise_Positive
                                        : InvertedValue.Clockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Brake));

        driveControlConfig.kP = SwerveModuleConstants.kTalonFXGains.drive().P();
        driveControlConfig.kI = SwerveModuleConstants.kTalonFXGains.drive().I();
        driveControlConfig.kD = SwerveModuleConstants.kTalonFXGains.drive().D();
        driveControlConfig.kV = SwerveModuleConstants.kTalonFXGains.driveV();

        turnConfig.withCurrentLimits(
                new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(120)
                        .withStatorCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(40)
                        .withSupplyCurrentLimitEnable(true));
        turnConfig.withMotorOutput(
                new MotorOutputConfigs()
                        .withInverted(
                                preset.turnInverted
                                        ? InvertedValue.CounterClockwise_Positive
                                        : InvertedValue.Clockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Brake));

        turnControlConfig.kP = SwerveModuleConstants.kTalonFXGains.turn().P();
        turnControlConfig.kI = SwerveModuleConstants.kTalonFXGains.turn().I();
        turnControlConfig.kD = SwerveModuleConstants.kTalonFXGains.turn().D();

        absoluteEncoder = new AnalogInput(preset.encoderPort);
        encoderOffset = preset.encoderOffset;
        turnMotor.setPosition(getAbsoluteRotation().getRotations() / preset.turnRot2Rads());

        driveMotor.getConfigurator().apply(driveConfig);
        turnMotor.getConfigurator().apply(turnConfig);

        driveMotor.getConfigurator().apply(driveControlConfig);
        turnMotor.getConfigurator().apply(turnControlConfig);
    }

    public void setVelocity(double velocity) {
        driveMotor.setControl(new VelocityVoltage(velocity / preset.driveRPM2MetersPerSec() / 60));
    }

    public void setRotation(Rotation2d rotation) {
        turnMotor.setControl(new PositionVoltage(rotation.getRadians() / preset.turnRot2Rads()));
    }

    public void resetEncoder() {
        turnMotor.setPosition(getAbsoluteRotation().getRotations() / preset.turnRot2Rads());
    }

    public void stop() {
        turnMotor.set(0);
        driveMotor.set(0);
    }

    public double getVelocity() {
        return driveMotor.getVelocity().getValueAsDouble() * 60 * preset.driveRPM2MetersPerSec();
    }

    public Rotation2d getAbsoluteRotation() {
        double angle = absoluteEncoder.getAverageVoltage() / RobotController.getVoltage5V();
        angle = Units.rotationsToRadians(angle);
        angle += encoderOffset;
        angle = MathUtil.inputModulus(angle, -Math.PI, Math.PI);
        return new Rotation2d(angle);
    }

    public Rotation2d getRotation() {
        return new Rotation2d(turnMotor.getPosition().getValueAsDouble() * preset.turnRot2Rads());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                driveMotor.getPosition().getValueAsDouble() * preset.driveRot2Meters(),
                getRotation());
    }
}
