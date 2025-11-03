package frc.robot.commands.DefaultCommands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AutoAiming;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ArmSubsystems.*;

public class ShooterMotorsCmd extends Command {
    // Suppliers are used so we can get constant updates to the values
    private Double shooterMotorsSpeed;
    private Supplier<Boolean> reversed;
    private Supplier<Pose2d> robotPose;
    private ShootingMotorSubsystem shootingMotorSubsystem;;

    public ShooterMotorsCmd(ShootingMotorSubsystem shootingMotorSubsystem, Supplier<Boolean> reversed,
            Supplier<Pose2d> robotPose) {
        this.shootingMotorSubsystem = shootingMotorSubsystem;
        this.reversed = reversed;
        this.robotPose = robotPose;
        addRequirements(shootingMotorSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();
        // this is a child class which inherits some code so we need to call the
        // constructor
        // of the parent class in the 1st line
    }

    @Override
    public void execute() {
        // runs the shooter motor at 75% speed when we fire in speaker and 50% for the
        // amp

        shooterMotorsSpeed = 0.0;
        shooterMotorsSpeed = reversed.get() ? -0.2 : shooterMotorsSpeed;
        shootingMotorSubsystem.runShooterMotors(shooterMotorsSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
