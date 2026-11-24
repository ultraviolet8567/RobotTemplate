package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.drive.Odometry.Odometry;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.util.MathHelper;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ManualTeleOp extends Command {
    private final Swerve swerve;
    private final Odometry odometry;
    private final CommandXboxController controller;
    private final Supplier<Double> leftX, leftY, rightX, rightY, driveSpeed, turnSpeed;
    private final Supplier<Boolean> buttonX, leftBumper;
    private final Timer inertialBuffer;
    private final PIDController pidController;

    public ManualTeleOp(
            Swerve swerve,
            Odometry odometry,
            CommandXboxController controller,
            Supplier<Double> driveSpeed,
            Supplier<Double> turnSpeed) {
        this.swerve = swerve;
        this.odometry = odometry;
        this.controller = controller;

        this.leftX = () -> -this.controller.getLeftY();
        this.leftY = () -> -this.controller.getLeftX();
        this.rightX = () -> -this.controller.getRightY();
        this.rightY = () -> -this.controller.getRightX();
        this.buttonX = () -> this.controller.x().getAsBoolean();
        this.leftBumper = () -> this.controller.leftBumper().getAsBoolean();
        this.driveSpeed = driveSpeed;
        this.turnSpeed = turnSpeed;

        this.inertialBuffer = new Timer();
        this.inertialBuffer.start();

        this.pidController = new PIDController(0.6, 0, 0);

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        double xVel = leftX.get();
        double yVel = leftY.get();
        double xTurn = rightX.get();
        double yTurn = rightY.get();
        double drive = driveSpeed.get();
        double turn = turnSpeed.get();
        Rotation2d heading = odometry.getHeading();

        if (Math.abs(xVel) < OIConstants.kDeadband) xVel = 0;
        if (Math.abs(yVel) < OIConstants.kDeadband) yVel = 0;
        if (Math.abs(xTurn) < OIConstants.kDeadband) xTurn = 0;
        if (Math.abs(yTurn) < OIConstants.kDeadband) yTurn = 0;

        double turnVel = yTurn;
        double angle = Math.atan2(yTurn, xTurn);
        Rotation2d error = MathHelper.minDist360(odometry.getHeading(), new Rotation2d(angle));

        if (leftBumper.get()) {
            turnVel = 0;
            if (Math.abs(error.getRadians()) > DriveConstants.kAllowedTurnError) {
                double strength = MathHelper.length(xTurn, yTurn);
                // turnVel = pidController.calculate(error.getRadians(), 0) * strength;

                Logger.recordOutput("Control/Angle", angle);
                Logger.recordOutput("Control/Diff", error);
                Logger.recordOutput("Control/Strength", strength);
            }
        }

        xVel = Math.pow(xVel, 3.0);
        yVel = Math.pow(yVel, 3.0);

        drive =
                MathUtil.clamp(
                        drive, -DriveConstants.kDriveMaxSpeed, DriveConstants.kDriveMaxSpeed);
        turn = MathUtil.clamp(drive, -DriveConstants.kTurnMaxSpeed, DriveConstants.kTurnMaxSpeed);

        if (buttonX.get()) {
            swerve.lockWheels();
            return;
        }

        if (xVel == 0 && yVel == 0 && turnVel == 0) {
            if (inertialBuffer.hasElapsed(0.5)) {
                swerve.holdPosition();
                return;
            }
        } else {
            inertialBuffer.restart();
        }
        swerve.updatePositions();

        ChassisSpeeds chassisSpeed;
        if (FieldConstants.fieldOriented) {
            chassisSpeed =
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            xVel * drive, yVel * drive, turnVel * turn, heading);
        } else {
            chassisSpeed = new ChassisSpeeds(xVel * drive, yVel * drive, turnVel * turn);
        }
        swerve.setStates(chassisSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }
}
