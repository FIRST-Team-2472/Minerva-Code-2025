package frc.robot.commands.DefaultCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AutoAiming;
import frc.robot.Constants.ArmMotorsConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.ArmSubsystems.*;

public class PitchMotorCmd extends Command {
    // Suppliers are used so we can get constant updates to the values
    private Supplier<Double> pitchMotor;
    private Double pitchMotorSpeed;
    private Supplier<Boolean> intakeButton;
    private Supplier<Pose2d> robotPose;
    private PitchMotorSubsystem armPitchSubsystem;
    private Timer timer = new Timer();

    public PitchMotorCmd(PitchMotorSubsystem armPitchSubsystem, Supplier<Double> pitchMotor,
            Supplier<Boolean> intakeButton, Supplier<Pose2d> robotPose) {
        this.pitchMotor = pitchMotor;
        this.armPitchSubsystem = armPitchSubsystem;
        this.intakeButton = intakeButton;
        this.robotPose = robotPose;
        addRequirements(armPitchSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();
        // this is a child class which inherits some code so we need to call the
        // constructor
        // of the parent class in the 1st line
        timer.start();
    }

    @Override
    public void execute() {
        // constantly update the pitch motor input
        if(armPitchSubsystem.getConstantAim() && AutoAiming.getSmartDistance(robotPose.get()) < OperatorConstants.autoAimDistance){

            double angleDeg = AutoAiming.getPitch(robotPose.get());
            armPitchSubsystem.runPitchMotorWithFasterKP(angleDeg);
        }else{
            pitchMotorSpeed = pitchMotor.get();

            // applies a deadband
            if (pitchMotorSpeed < OIConstants.kArmDeadband && pitchMotorSpeed > -OIConstants.kArmDeadband)
                pitchMotorSpeed = 0.0;
            
            pitchMotorSpeed *= 0.45;// slows down the arm

            if (!intakeButton.get())
                armPitchSubsystem.runPitchMotor(pitchMotorSpeed);
            else{
                //if(armPitchSubsystem.getEncoderDeg() > 5){
                //    armPitchSubsystem.runPitchMotorWithFasterKP(ArmMotorsConstants.PitchMotor.kPitchMotorIntakePresetAngle);
                //}else{
                //    armPitchSubsystem.runPitchMotor(pitchMotorSpeed, true);
                //}
            }

        }
        super.execute();
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
