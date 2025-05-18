package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotStatus;
import frc.robot.subsystems.ArmSubsystems.*;

public class ShootNoteCmd extends Command {

    private ShootingMotorSubsystem shooterSubsystem;
    private IntakeMotorSubsystem intakeSubsystem;
    private Timer overideTimer, timerTwo;
    private double speed;
    private int rpm;

    public ShootNoteCmd(ShootingMotorSubsystem shooterSubsystem, IntakeMotorSubsystem intakeSubsystem, double speed) {
        overideTimer = new Timer();
        timerTwo = new Timer();
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.speed = speed;
        addRequirements(shooterSubsystem);
        addRequirements(intakeSubsystem);
        rpm = 3500;
    }

    public ShootNoteCmd(ShootingMotorSubsystem shooterSubsystem, IntakeMotorSubsystem intakeSubsystem, double speed,
            int rpm) {
        this(shooterSubsystem, intakeSubsystem, speed);
        this.rpm = rpm;
    }

    @Override
    public void initialize() {
        overideTimer.stop();
        overideTimer.reset();
        overideTimer.start();

        timerTwo.stop();
        timerTwo.reset();
        System.out.println("running");
        System.out.println("override timer: " + overideTimer.get());
        System.out.println("" + -1 * rpm);
    }

    @Override
    public void execute() {
        shooterSubsystem.runShooterMotors(speed);

        if (shooterSubsystem.getShooterSpeed() < -rpm || overideTimer.hasElapsed(2.5)) {
            
            timerTwo.start();

            intakeSubsystem.runPushMotor(1);
            intakeSubsystem.runIntakeMotors(1);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.runShooterMotors(0);
        intakeSubsystem.runPushMotor(0);
        intakeSubsystem.runIntakeMotors(0);
        System.out.println("interrupted");
    }

    @Override
    public boolean isFinished() {
        return overideTimer.hasElapsed(3) || timerTwo.hasElapsed(0.6)|| RobotStatus.armAngle > 60;
    }
}
