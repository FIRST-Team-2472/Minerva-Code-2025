package frc.robot.commands.DefaultCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotStatus;
import frc.robot.subsystems.ArmSubsystems.*;


public class IntakeMotorCmd extends Command {
    // Suppliers are used so we can get constant updates to the values
    private Double intakeMotorsSpeed, pushMotorSpeed;
    private Supplier<Boolean> intakeMotorsRunning,  reversed;
    private boolean sensed;
    private IntakeMotorSubsystem intakeMotorSubsystem;
    private Timer timer = new Timer();

    public IntakeMotorCmd(IntakeMotorSubsystem intakeMotorSubsystem,
         Supplier<Boolean> intakeMotorsRunning, Supplier<Boolean> reversed){
        sensed = false;
        this.intakeMotorsRunning = intakeMotorsRunning;
        this.intakeMotorSubsystem = intakeMotorSubsystem;
        this.reversed = reversed;
        addRequirements(intakeMotorSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();
        //this is a child class which inherits some code so we need to call the constructor
        // of the parent class in the 1st line
        timer.start();
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("note sensor", intakeMotorSubsystem.getPhotoElectricSensor());
         if(intakeMotorSubsystem.getPhotoElectricSensor()){
            sensed = true;
            timer.reset();
        } 
        if(timer.hasElapsed(2)){
            sensed = false;
        }
        
        //runs the push motor until it hit the sensor
        pushMotorSpeed = 0.0;
        pushMotorSpeed = reversed.get() ? -.2 : pushMotorSpeed;
        if (RobotStatus.armAngle > 60)pushMotorSpeed = intakeMotorsRunning.get() && !sensed ? .3 : pushMotorSpeed;
        intakeMotorSubsystem.runPushMotor(pushMotorSpeed);


        //runs the intake motors until the sensor is triggered
        intakeMotorsSpeed = 0.0;
        intakeMotorsSpeed = reversed.get() ? -0.2 : intakeMotorsSpeed;
        intakeMotorsSpeed = intakeMotorsRunning.get() && !sensed ? .3 : intakeMotorsSpeed;
        intakeMotorSubsystem.runIntakeMotors(intakeMotorsSpeed);
        
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
