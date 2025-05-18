package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ArmMotorsConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ConstantAimToggleCmd;
import frc.robot.commands.FastAutoAimCmd;
import frc.robot.commands.IntakeNoteCmd;
import frc.robot.commands.OverrideCmd;
import frc.robot.commands.ResetHeadingCmd;
import frc.robot.commands.SetArmPitchCmd;
import frc.robot.commands.SwerveRotateToAngle;
import frc.robot.commands.ShootNoteCmd;
import frc.robot.commands.SmallPnuematicsCmd;
import frc.robot.commands.DefaultCommands.IntakeMotorCmd;
import frc.robot.commands.DefaultCommands.PitchMotorCmd;
import frc.robot.commands.DefaultCommands.PneumaticsCmd;
import frc.robot.commands.DefaultCommands.ShooterMotorsCmd;
import frc.robot.commands.DefaultCommands.SwerveJoystickCmd;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ArmSubsystems.IntakeMotorSubsystem;
import frc.robot.subsystems.ArmSubsystems.PitchMotorSubsystem;
import frc.robot.subsystems.ArmSubsystems.ShootingMotorSubsystem;

public class RobotContainer {
  private final String SPtwoNtwoNone = "3 notes in speaker Speaker 2 Note 2 - 1 - TESTED", SPtwoNtwo = "2 notes in speaker Seaker 2 Note 2 - TESTED",
  SPtwoNoneNtwoNthree = "4 notes in speaker from Speaker 2 Notes 1 - 2 - 3", SPtwoNthreeNtwoNoneNfour = "4 in speaker Speaker 2 Notes 3 - 2 - 1 - 4",
  SPtwoNoneNfour = "3 in speaker Speaker 2 Notes 2 - 1 pick up 4", SPtwoNtwoNfour = "3 in speaker Speaker 2 Notes 2 - 4",
  SPoneNoneNfourRSPone = "3 in speaker Speaker one Notes 1 - 4", SPthreeNthreeNeightNseven = "3 in speaker Speaker 3 Notes 3 - 8 - 7",
  SPthreeNfourNfive = "3 in speaker Speaker 3 Notes 4 - 5", SPthreeNfiveNfour = "3 in speaker Speaker 3 Notes 5 - 4",
  SPthreeNthree = "2 in speaker Speaker 3 Note 3", SPoneNone = "2 in speaker Speaker 1 Note 1", 
  SPthreeNeightNseven = "3 in speaker Speaker 3 Notes 8 - 7", SpThreeNThreeNEight = "3 in speaker out of the way stage side", test = "test",
  justShoot = "just shoot";
  
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private final CommandSequences commandSequences = new CommandSequences();
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final PneumaticsSubsystem pneumaticsSubsystem = new PneumaticsSubsystem();


  
  private final PitchMotorSubsystem pitchMotorSubsystem = new PitchMotorSubsystem();
  private final IntakeMotorSubsystem intakeMotorSubsystem = new IntakeMotorSubsystem();
  private final ShootingMotorSubsystem shootingMotorSubsystem = new ShootingMotorSubsystem();

  //private final Limelights limelights = new Limelights(swerveSubsystem, armSubsystem);
  XboxController xbox = new XboxController(OperatorConstants.kXboxControllerPort);
  public static Joystick leftJoystick = new Joystick(OperatorConstants.kLeftJoyPort);
  public static Joystick rightJoystick = new Joystick(OperatorConstants.kRightJoyPort);
  
  

  public RobotContainer() {
    pitchMotorSubsystem.setDefaultCommand(new PitchMotorCmd(pitchMotorSubsystem, () -> -xbox.getRightY() * 0, () -> xbox.getRightTriggerAxis() > 0.5, () -> swerveSubsystem.getPose())); // Intake Motors
    intakeMotorSubsystem.setDefaultCommand(new IntakeMotorCmd(intakeMotorSubsystem, () -> xbox.getRightTriggerAxis() > 0.5,
    () -> xbox.getYButton()));
    shootingMotorSubsystem.setDefaultCommand(new ShooterMotorsCmd(shootingMotorSubsystem, () -> xbox.getYButton(), () -> swerveSubsystem.getPose()));

    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(swerveSubsystem, 
      ()-> xbox.getLeftY(),
      ()-> xbox.getLeftX(),
      ()-> -xbox.getRightX(),
      ()-> false
    ));

    pneumaticsSubsystem.setDefaultCommand(new PneumaticsCmd(pneumaticsSubsystem));
    
    configureBindings();


    m_chooser.addOption(SPtwoNtwoNone, SPtwoNtwoNone);
    m_chooser.addOption(SPtwoNtwo, SPtwoNtwo);
    m_chooser.addOption(SPtwoNoneNtwoNthree, SPtwoNoneNtwoNthree);
    m_chooser.addOption(SPtwoNthreeNtwoNoneNfour, SPtwoNthreeNtwoNoneNfour);
    m_chooser.addOption(SPtwoNoneNfour, SPtwoNoneNfour);
    m_chooser.addOption(SPtwoNtwoNfour, SPtwoNtwoNfour);
    m_chooser.addOption(SPoneNoneNfourRSPone, SPoneNoneNfourRSPone);
    m_chooser.addOption(SPthreeNthreeNeightNseven, SPthreeNthreeNeightNseven);
    m_chooser.addOption(SPthreeNfourNfive, SPthreeNfourNfive);
    m_chooser.addOption(SPthreeNfiveNfour, SPthreeNfiveNfour);
    m_chooser.addOption(SPthreeNthree, SPthreeNthree);
    m_chooser.addOption(SPoneNone, SPoneNone);
    m_chooser.addOption(SPthreeNeightNseven, SPthreeNeightNseven);
    m_chooser.addOption(SpThreeNThreeNEight, SpThreeNThreeNEight);
    m_chooser.addOption(test, test);
    m_chooser.addOption(justShoot, justShoot);
    

    ShuffleboardTab driverBoard = Shuffleboard.getTab("Driver Board");
    driverBoard.add("Auto choices", m_chooser).withWidget(BuiltInWidgets.kComboBoxChooser);
    driverBoard.addCamera("Limelight Stream Intake", "limelight_intake", "mjpg:http://limelight-intake.local:5800").withSize(4,4);
    driverBoard.addCamera("Limelight Stream Shooter", "limelight_shooter", "mjpg:http://limelight-shooter.local:5800").withSize(4,4);

    //warning a name change will break auto paths because pathplanner will not update it
   
        System.out.println("team " + SwerveSubsystem.isOnRed());
  }

  private void configureBindings() {
    new JoystickButton(rightJoystick, 1).onTrue(new InstantCommand(swerveSubsystem :: zeroHeading));
    new JoystickButton(rightJoystick, 3).onTrue(new OverrideCmd(swerveSubsystem, intakeMotorSubsystem, pitchMotorSubsystem, shootingMotorSubsystem));
    new JoystickButton(rightJoystick, 13).onTrue(new InstantCommand(swerveSubsystem :: disableCams));
    new JoystickButton(rightJoystick, 2).onTrue(new ShootNoteCmd(shootingMotorSubsystem, intakeMotorSubsystem, 0.9, 2000));

    new CommandXboxController(OperatorConstants.kXboxControllerPort).leftBumper().onTrue(new SmallPnuematicsCmd(pneumaticsSubsystem));
    new CommandXboxController(OperatorConstants.kXboxControllerPort).rightBumper().onTrue(new InstantCommand(pneumaticsSubsystem :: toggleBigpneumatics));

    new CommandXboxController(OperatorConstants.kXboxControllerPort).b().onTrue(new SetArmPitchCmd(pitchMotorSubsystem, 40));
    new CommandXboxController(OperatorConstants.kXboxControllerPort).a().onTrue(new SetArmPitchCmd(pitchMotorSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorIntakePresetAngle));
    
    //new CommandXboxController(OperatorConstants.kXboxControllerPort).x().onTrue(new SetArmPitchCmd(pitchMotorSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorAmpPresetAngle));
    new CommandXboxController(OperatorConstants.kXboxControllerPort).rightTrigger(0.5).onTrue(new ShootNoteCmd(shootingMotorSubsystem, intakeMotorSubsystem, 0.9, 4000));

      //new CommandXboxController(OperatorConstants.kXboxControllerPort).leftTrigger(0.5).onTrue(new ShootNoteCmd(shootingMotorSubsystem, intakeMotorSubsystem, 0.4, 0));
    //new CommandXboxController(OperatorConstants.kXboxControllerPort).y().onTrue(new SetArmPitchCmd(armSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorStandbyPresetAngle));
    new CommandXboxController(OperatorConstants.kXboxControllerPort).start().onTrue(new FastAutoAimCmd(pitchMotorSubsystem, swerveSubsystem, shootingMotorSubsystem, intakeMotorSubsystem));
    new CommandXboxController(OperatorConstants.kXboxControllerPort).back().onTrue(new ConstantAimToggleCmd(swerveSubsystem, pitchMotorSubsystem, shootingMotorSubsystem));
  }

  public Command getAutonomousCommand() {
    System.out.println("Autos Begun");

    return null;
  }

  public void logSwerve() {

    SmartDashboard.putNumber("Heading", swerveSubsystem.getHeading());
    SmartDashboard.putString("Robot Location", swerveSubsystem.getPose().getTranslation().toString());
    SmartDashboard.putNumber("frontLeft Encoder", swerveSubsystem.getFLAbsEncoder());
    SmartDashboard.putNumber("frontRight Encoder", swerveSubsystem.getFRAbsEncoder());
    SmartDashboard.putNumber("BackLeft Encoder", swerveSubsystem.getBLAbsEncoder());
    SmartDashboard.putNumber("BackRight Encoder", swerveSubsystem.getBRAbsEncoder());
    SmartDashboard.putNumber("Shooter speed", shootingMotorSubsystem.getShooterSpeed());
    SmartDashboard.putNumber("Rotation", swerveSubsystem.getRotation2d().getDegrees());
    //SmartDashboard.putNumber("Arm Encoder", armSubsystem.getAbsoluteEncoder());
  }

}
