package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.commands.ManualTeleOp;
import frc.robot.commands.Rumble;
import frc.robot.subsystems.drive.Autos;
import frc.robot.subsystems.drive.Odometry;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.drive.SwerveModule.SwerveModuleIOSim;
import frc.robot.subsystems.drive.SwerveModule.SwerveModuleIOSparkMax;

public class RobotContainer {
    private final Swerve swerve;
    private final Odometry odometry;
    private final Autos autos;

    private static final CommandXboxController driver =
            new CommandXboxController(Constants.OIConstants.kDriverControllerPort);
    private static final CommandXboxController operator =
            new CommandXboxController(Constants.OIConstants.kOperatorControllerPort);

    public RobotContainer() {
        if (RobotBase.isSimulation()) {
            swerve =
                    new Swerve(
                            new SwerveModuleIOSim(
                                    DriveConstants.kModuleFL,
                                    SwerveModuleConstants.kPresetNEO,
                                    SwerveModuleConstants.kPresetNEO),
                            new SwerveModuleIOSim(
                                    DriveConstants.kModuleFR,
                                    SwerveModuleConstants.kPresetNEO,
                                    SwerveModuleConstants.kPresetNEO),
                            new SwerveModuleIOSim(
                                    DriveConstants.kModuleBL,
                                    SwerveModuleConstants.kPresetNEO,
                                    SwerveModuleConstants.kPresetNEO),
                            new SwerveModuleIOSim(
                                    DriveConstants.kModuleBR,
                                    SwerveModuleConstants.kPresetNEO,
                                    SwerveModuleConstants.kPresetNEO));
        } else {
            swerve =
                    new Swerve(
                            new SwerveModuleIOSparkMax(DriveConstants.kModuleFL),
                            new SwerveModuleIOSparkMax(DriveConstants.kModuleFR),
                            new SwerveModuleIOSparkMax(DriveConstants.kModuleBL),
                            new SwerveModuleIOSparkMax(DriveConstants.kModuleBR));
        }

        odometry = new Odometry(swerve);
        autos = new Autos();

        configureAutos();
        configureBindings();
    }

    private void configureAutos() {}

    private void configureBindings() {
        swerve.setDefaultCommand(
                new ManualTeleOp(
                        swerve,
                        odometry,
                        driver,
                        () ->
                                driver.getHID().getRightBumperButton()
                                        ? DriveConstants.kDriveSlowSpeed
                                        : DriveConstants.kDriveSpeed,
                        () ->
                                driver.getHID().getRightBumperButton()
                                        ? DriveConstants.kTurnSlowSpeed
                                        : DriveConstants.kTurnSpeed));
        driver.rightBumper()
                .whileTrue(
                        new Rumble(driver, RumbleType.kRightRumble, OIConstants.kRumbleStrength));
    }

    public static CommandXboxController getDriver() {
        return driver;
    }

    public static CommandXboxController getOperator() {
        return operator;
    }
}
