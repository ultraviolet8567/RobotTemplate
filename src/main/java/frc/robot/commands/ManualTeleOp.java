package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.drive.Odometry.Odometry;
import frc.robot.subsystems.drive.Swerve;
import java.util.function.Supplier;

public class ManualTeleOp extends Command {
    private final Swerve swerve;
    private final Odometry odometry;
    private final CommandXboxController controller;
    private final Supplier<Double> leftX, leftY, rightX, rightY, driveSpeed, turnSpeed;
    private final Supplier<Boolean> xButton;

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
        this.xButton = () -> this.controller.x().getAsBoolean();
        this.driveSpeed = driveSpeed;
        this.turnSpeed = turnSpeed;

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

        double turnVel = xTurn;

        if (Math.abs(xVel) < OIConstants.kDeadband) xVel = 0;
        if (Math.abs(yVel) < OIConstants.kDeadband) yVel = 0;
        if (Math.abs(turnVel) < OIConstants.kDeadband) turnVel = 0;

        xVel = Math.pow(xVel, 3.0);
        yVel = Math.pow(yVel, 3.0);

        drive =
                MathUtil.clamp(
                        drive, -DriveConstants.kDriveMaxSpeed, DriveConstants.kDriveMaxSpeed);
        turn = MathUtil.clamp(drive, -DriveConstants.kTurnMaxSpeed, DriveConstants.kTurnMaxSpeed);

        if (xVel == 0 && yVel == 0 && turnVel == 0) {
            swerve.holdPosition();
        } else {
            swerve.updatePositions();
        }

        if (xButton.get()) {
            swerve.lockWheels();
        } else {
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
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }
}
