package frc.robot.commands.DefaultCommands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AutoAiming;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.TargetPosConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.swerveExtras.NewNewAccelLimiter;

public class SwerveJoystickCmd extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> slowed;
    private final NewNewAccelLimiter xLimiter, yLimiter;
    private PIDController turningController;

    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
            Supplier<Boolean> slowed) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.slowed = slowed;

        xLimiter = new NewNewAccelLimiter(.02, .1);
        yLimiter = new NewNewAccelLimiter(.02, .1);

        addRequirements(swerveSubsystem);

        turningController = new PIDController(0.03, 0.1, 0);
    }

    @Override
    public void initialize() {
        swerveSubsystem.initializeJoystickRunFromField();
    }

    public static double input_2_speed(double x) {
        return 19.4175 * Math.pow(x, 13) - 103.7677 * Math.pow(x, 11) + 195.0857 * Math.pow(x, 9)
                - 165.5452 * Math.pow(x, 7) + 61.8185 * Math.pow(x, 5) - 6.5099 * Math.pow(x, 3) + 0.5009 * x;
    }

    @Override
    public void execute() {
        // get joystick values
        double xInput = SwerveSubsystem.isOnRed() ? -xSpdFunction.get() : -xSpdFunction.get();
        double yInput = SwerveSubsystem.isOnRed() ? -ySpdFunction.get() : -ySpdFunction.get();
        double turningSpeed = turningSpdFunction.get();

        double xSpeed;
        double ySpeed;

            xSpeed = xInput;
            ySpeed = yInput;


        // deadband (area that doesnt actually result in an input)
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband
                ? (!slowed.get() ? xSpeed : xSpeed * OperatorConstants.kSlowedSpeed)
                : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband
                ? (!slowed.get() ? ySpeed : ySpeed * OperatorConstants.kSlowedSpeed)
                : 0.0;

        if (swerveSubsystem.getConstantAim()
                && AutoAiming.getSmartDistance(swerveSubsystem.getPose()) < OperatorConstants.autoAimDistance) {

            Rotation2d angleDifference = swerveSubsystem.getPose().getRotation()
                    .minus(Rotation2d.fromDegrees(AutoAiming.getYaw(swerveSubsystem.getPose())));
            turningSpeed = MathUtil.clamp(turningController.calculate(angleDifference.getRadians(), 0), -1, 1)
                    * TargetPosConstants.kMaxAngularSpeed / 2;

            // turningSpeed += Math.copySign(TargetPosConstants.kMinAngularSpeedRadians,
            // turningSpeed);
        } else {
            turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;
        }

        xSpeed = xLimiter.calculate(xSpeed);
        ySpeed = yLimiter.calculate(ySpeed);
        swerveSubsystem.executeJoystickRunFromField(xSpeed, ySpeed, turningSpeed);

    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
