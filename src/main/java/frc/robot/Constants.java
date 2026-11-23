package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static class FieldConstants {
        public static final boolean fieldOriented = true;
    }

    public static class OIConstants {
        public static final ControllerType controllerTypeDriver = ControllerType.XBOX;
        public static final ControllerType controllerTypeOperator = ControllerType.XBOX;

        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;

        public static final double kDeadband = 0.11;
        public static final double kRumbleStrength = 0.025;
    }

    public static class SwerveModulePreset {
        public final String name;
        public final int drivePort, turnPort, encoderPort;
        public final double driveRatio, encoderOffset;
        public final boolean driveInverted, turnInverted;

        public SwerveModulePreset(
                String name,
                int drivePort,
                int turnPort,
                boolean driveInverted,
                boolean turnInverted,
                int encoderPort,
                double encoderOffset) {

            this.name = name;

            this.drivePort = drivePort;
            this.turnPort = turnPort;

            this.encoderPort = encoderPort;
            this.encoderOffset = encoderOffset;

            this.driveRatio = SwerveModuleConstants.kDriveGearRatios[DriveConstants.kGear];
            this.driveInverted = driveInverted;
            this.turnInverted = turnInverted;
        }

        // units: rot -> m
        public double driveRot2Meters() {
            return driveRatio * Math.PI * SwerveModuleConstants.kWheelDiameter;
        }

        // units: rot -> rad
        public double turnRot2Rads() {
            return Units.rotationsToRadians(SwerveModuleConstants.kTurnGearRatio);
        }

        // units: rot/min -> m/s
        public double driveRPM2MetersPerSec() {
            return driveRot2Meters() / 60;
        }

        // units: rot/min -> rad/s
        public double turnRPM2RadsPerSec() {
            return turnRot2Rads() / 60;
        }
    }

    public static class MotorPreset {
        public final DCMotor motor;
        public final double momentInertia;

        public MotorPreset(DCMotor motor, double momentInertia) {
            this.motor = motor;
            this.momentInertia = momentInertia;
        }

        public LinearSystem<N2, N1, N2> getLinearSystem(double gearing) {
            return LinearSystemId.createDCMotorSystem(motor, momentInertia, gearing);
        }
    }

    public static class DriveConstants {
        public static final int kGear = 1;
        public static final GearboxType kGearbox = GearboxType.MK4N;

        // units: m/s
        public static final double kDriveMaxSpeed = 5;
        public static final double kDriveSpeed = 5;
        public static final double kDriveSlowSpeed = 1.5;

        // units: rad/s
        public static final double kTurnMaxSpeed = 5 * Math.PI;
        public static final double kTurnSpeed = 3 * Math.PI;
        public static final double kTurnSlowSpeed = 1 * Math.PI;

        // units: m
        public static final double kTrackWidth = Units.inchesToMeters(21.75);
        public static final double kWheelBase = Units.inchesToMeters(21.75);

        public static final SwerveDriveKinematics kDriveKinematics =
                new SwerveDriveKinematics(
                        new Translation2d(kWheelBase / 2, kTrackWidth / 2), // Front left (+/+)
                        new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // Front right (+/-)
                        new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // Back left (-/+)
                        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); // Back right (-/-)

        public static final SwerveModulePreset kModuleFL =
                new SwerveModulePreset(
                        "SwerveModuleFL",
                        CAN.kDriveMotorPortFL,
                        CAN.kTurnMotorPortFL,
                        true,
                        true,
                        0,
                        0.714 - 1.325);
        public static final SwerveModulePreset kModuleFR =
                new SwerveModulePreset(
                        "SwerveModuleFR",
                        CAN.kDriveMotorPortFR,
                        CAN.kTurnMotorPortFR,
                        true,
                        true,
                        1,
                        -1.745 - 0.009);
        public static final SwerveModulePreset kModuleBL =
                new SwerveModulePreset(
                        "SwerveModuleBL",
                        CAN.kDriveMotorPortBL,
                        CAN.kTurnMotorPortBL,
                        false,
                        true,
                        2,
                        -0.106 + 0.014);
        public static final SwerveModulePreset kModuleBR =
                new SwerveModulePreset(
                        "SwerveModuleBR",
                        CAN.kDriveMotorPortBR,
                        CAN.kTurnMotorPortBR,
                        true,
                        true,
                        3,
                        0.048 + 0.412);

        /*
        // units: m/s
        public static final double kMaxSpeed = 5;
        // units: rad/s
        public static final double kMaxAngularSpeed = 3 * Math.PI;

        // units: m/s/s
        public static final double kMaxAcceleration = 5;
        // units: rad/s/s
        public static final double kMaxAngularAcceleration = 3 * Math.PI;
         */

        // units: kg
        public static final double kRobotMass = 50;
        // units: kg m^2
        public static final double kRobotMOI = 1.29;

        public static final SwerveModuleState[] kLockStates =
                new SwerveModuleState[] {
                    new SwerveModuleState(0, new Rotation2d(Math.PI / 4)),
                    new SwerveModuleState(0, new Rotation2d(-Math.PI / 4)),
                    new SwerveModuleState(0, new Rotation2d(-Math.PI / 4)),
                    new SwerveModuleState(0, new Rotation2d(Math.PI / 4))
                };
    }

    public static record SwerveModuleGains(
            double turnP,
            double turnI,
            double turnD,
            double driveP,
            double driveI,
            double driveD,
            double driveV,
            double lockP,
            double lockI,
            double lockD) {}

    public static final class SwerveModuleConstants {
        // units: m
        public static final double kWheelDiameter = Units.inchesToMeters(3.75);
        // units: m/s
        public static final double kMaxSpeed = 12;

        // units m/s
        public static final double kAllowedDriveError = 0.01;
        // units m
        public static final double kAllowedPositionError = 0.005;
        public static final double kPositionCorrectionRate = 0.00001;
        // units: rad
        public static final double kAllowedTurnError = 0.1;

        public static final double[] kMK5NDriveGearRatios = {
            1, // Filler
            225.0 / 32.0,
            675.0 / 112.0,
            675.0 / 128.0
        };

        public static final double[] kMK4NDriveGearRatios = {
            1, // Filler
            1 / 5.9
        };

        public static final double kMK5NTurnGearRatio = 287.0 / 11.0;
        public static final double kMK4NTurnGearRatio = 4.0 / 75.0;

        public static final double[] kDriveGearRatios = kMK4NDriveGearRatios;
        public static final double kTurnGearRatio = kMK4NTurnGearRatio;

        public static final MotorPreset kPresetNEO = new MotorPreset(DCMotor.getNEO(1), 0.005);
        public static final MotorPreset kPresetKrakenX60 =
                new MotorPreset(DCMotor.getKrakenX60(1), 0.005);
        public static final MotorPreset kPresetKrakenX44 =
                new MotorPreset(DCMotor.getKrakenX44(1), 0.005);

        public static final SwerveModuleGains kSparkMaxGains =
                new SwerveModuleGains(1, 0, 1, 0.05, 0, 1, 0.2, 4, 0, 12);
        public static final SwerveModuleGains kTalonFXGains =
                new SwerveModuleGains(1, 0, 1, 0.05, 0, 0, 0.2, 0.1, 0, 0.2);
    }

    public static class CAN {
        public static final int kDriveMotorPortFL = 10;
        public static final int kTurnMotorPortFL = 20;

        public static final int kDriveMotorPortFR = 11;
        public static final int kTurnMotorPortFR = 21;

        public static final int kDriveMotorPortBL = 12;
        public static final int kTurnMotorPortBL = 22;

        public static final int kDriveMotorPortBR = 13;
        public static final int kTurnMotorPortBR = 23;

        public static final int kPigeonPort = 31;
    }

    public static enum ControllerType {
        XBOX,
        LOGITECH,
        JOYSTICK
    }

    public static enum GearboxType {
        MK4N,
        MK5N
    }
}
