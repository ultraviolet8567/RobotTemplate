package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.drive.Odometry;
import frc.robot.subsystems.drive.Swerve;
import java.util.function.Supplier;

public class ManualTeleOp extends Command {
    private final Swerve swerve;
    private final Odometry odometry;
    private final CommandXboxController controller;
    private final Supplier<Double> xControl, yControl, turnControl, driveSpeed, turnSpeed;

    public ManualTeleOp(
            Swerve swerve,
            Odometry odometry,
            CommandXboxController controller,
            Supplier<Double> driveSpeed,
            Supplier<Double> turnSpeed) {
        this.swerve = swerve;
        this.odometry = odometry;
        this.controller = controller;

        this.xControl = () -> -this.controller.getLeftY();
        this.yControl = () -> -this.controller.getLeftX();
        this.turnControl = () -> -this.controller.getRightX();
        this.driveSpeed = driveSpeed;
        this.turnSpeed = turnSpeed;

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        double xVel = xControl.get();
        double yVel = yControl.get();
        double turnVel = turnControl.get();
        double drive = driveSpeed.get();
        double turn = turnSpeed.get();
        Rotation2d heading = odometry.getHeading();

        drive =
                MathUtil.clamp(
                        drive, -DriveConstants.kDriveMaxSpeed, DriveConstants.kDriveMaxSpeed);
        turn = MathUtil.clamp(drive, -DriveConstants.kTurnMaxSpeed, DriveConstants.kTurnMaxSpeed);

        if (Math.abs(xVel) < OIConstants.kDeadband) xVel = 0;
        if (Math.abs(yVel) < OIConstants.kDeadband) yVel = 0;
        if (Math.abs(turnVel) < OIConstants.kDeadband) turnVel = 0;

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
