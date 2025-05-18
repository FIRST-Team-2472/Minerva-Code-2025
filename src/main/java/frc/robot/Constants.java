package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static class OperatorConstants {
    public static final int kLeftJoyPort = 1;
    public static final int kRightJoyPort = 2;
    public static final int kXboxControllerPort = 3;
    public static final double kSlowedSpeed = 0.3;
    public static final double autoAimDistance = 3.5;
  }

  public static final class ModuleConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 1 / 6.75;
    public static final double kTurningMotorGearRatio = 1 / 12.8;
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
    public static final double kPTurning = 0.5;
  }

  public static final class SensorConstants {
    public static final int kPigeonID = 13;
    public static final double sizeOfFieldMeters = 16.5;
    public static final int kPhotoElectricSensorID = 2;
  }

  public static final class OIConstants {
    public static final double kDeadband = 0.15;
    public static final double kArmDeadband = 0.1;
    public static final int kLeftDriverYAxis = 1; // conflicts with other constants
    public static final int kLeftDriverXAxis = 0; // conflicts with other constants
    public static final int kRightDriverRotAxis = 0; // conflicts with other constants
  }

  public static final class DriveConstants {

    public static final double kTrackWidth = Units.inchesToMeters(21);

    public static final double kWheelBase = Units.inchesToMeters(23);

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final int kFrontLeftDriveMotorPort = 55;
    public static final int kFrontLeftTurningMotorPort = 54;
    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final int kFrontLeftDriveAbsoluteEncoderPort = 61;
    public static final double kFrontLeftDriveAbsoluteEncoderOffsetAng = 0.25;
    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;

    public static final int kFrontRightDriveMotorPort = 53;
    public static final int kFrontRightTurningMotorPort = 51;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final int kFrontRightDriveAbsoluteEncoderPort = 62; // conflicts with other constants
    public static final double kFrontRightDriveAbsoluteEncoderOffsetAng = 0.46;
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;

    public static final int kBackLeftDriveMotorPort = 43;
    public static final int kBackLeftTurningMotorPort = 50;
    public static final boolean kBackLeftDriveEncoderReversed = false;
    public static final boolean kBackLeftTurningEncoderReversed = false;
    public static final int kBackLeftDriveAbsoluteEncoderPort = 60; // conflicts with other constants
    public static final double kBackLeftDriveAbsoluteEncoderOffsetAng = 0.03;
    public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;

    public static final int kBackRightDriveMotorPort = 56;
    public static final int kBackRightTurningMotorPort = 57;
    public static final boolean kBackRightDriveEncoderReversed = false;
    public static final boolean kBackRightTurningEncoderReversed = false;
    public static final int kBackRightDriveAbsoluteEncoderPort = 59;
    public static final double kBackRightDriveAbsoluteEncoderOffsetAng = 0.35;
    public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

    public static final double kPhysicalMaxSpeedMetersPerSecond = 3;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 7 * 3 * Math.PI;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond
        / 10;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 20;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = Math.PI * 2;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 1;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.5;
    public static final double kPXController = 1.5;
    public static final double kPYController = 1.5;
    public static final double kPThetaController = 1.5;
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxSpeedMetersPerSecond,
        kMaxAccelerationMetersPerSecondSquared);
    public static double kThetaController;

    public static final double TRANSLATION_KP = .7;
    public static final double TRANSLATION_KI = 0;
    public static final double TRANSLATION_KD = 0;

    public static final double ROTATION_KP = .7;
    public static final double ROTATION_KI = 0;
    public static final double ROTATION_KD = 0;

    public static final double MAX_MODULE_SPEED = 4.5;
    public static final double DRIVE_BASE_RADIUS_METERS = Math.hypot(DriveConstants.kTrackWidth,
        DriveConstants.kWheelBase) / 2;
  }

  public static class ArmMotorsConstants {

    public static class PitchMotor {
      public static final int kPitchMotorId = 58;
      public static final double kPitchMotorKP = 0.15;
      public static final int kPitchEncoderId = 0;
      public static final double kPitchEncoderOffset = -274;
      public static final double kPitchInternalEncoderConversionFactor = -((4 / 9) * 100); // -44.4444...
      public static final double kPitchBaseIdleForce = 0.052;
      public static final double kPitchEncoderForwardLimit = 10;
      public static final double kPitchEncoderReverseLimit = -88;
      public static final double kPitchMotorIntakePresetAngle = 90.5;
      public static final double kPitchMotorSpeakerPresetAngle = 78.0;
      public static final double kPitchMotorFarSpeakerPresetAngle = 60;
      public static final double kPitchMotorAmpPresetAngle = -7;
      public static final double kPitchMotorStandbyPresetAngle = 0.0;
    }

    public static class ShooterMotors {
      public static final int kTopShooterMotorId = 44; // conflicts with other constants
      public static final int kBottomShooterMotorId = 62;
    }

    public static class PushMotor {
      public static final int kPushMotorId = 52;
    }

    public static class IntakeMotors {
      public static final int kTopIntakeMotorId = 60;
      public static final int kBottomIntakeMotorId = 41; // conflicts with other constants
    }

  }

  public static final class TargetPosConstants {
    // Motion constants for target position drive mode
    public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
    public static final double kMaxAngularSpeed = //
        DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 16;

    public static final double kForwardMaxAcceleration = 2;
    public static final double kBackwardMaxAcceleration = -12;
    public static final double kMaxAngularAcceleration = Math.PI / 3;
    public static final double kBackwardAngularAcceleration = -Math.PI * 9;

    public static final double kMinAngularSpeedRadians = Math.PI / 16;
    public static final double kMinSpeedMetersPerSec = .2;

    public static final double kPDriveController = 1.9;
    public static final double kPAngleController = 0.3;
    public static final double kAcceptableDistanceError = 0.12;
    public static final double kAcceptableAngleError = 1.5;
}
  
public static final class PneumaticsConstants{
    public static final int kCompressorid = 1;

    public static final int kBigLeftPneumaticInflateChannel = 0;
    public static final int kBigLeftPneumaticDeflateChannel = 12;

    public static final int kSmallLeftPneumaticInflateChannel = 1;
    public static final int kSmallLeftPneumaticDeflateChannel = 13;

    public static final int kSmallRightPneumaticInflateChannel = 3;
    public static final int kSmallRightPneumaticDeflateChannel = 15;
    
    public static final int kBigRightPneumaticInflateChannel = 2;
    public static final int kBigRightPneumaticDeflateChannel = 14;
}
public static final class AutoAimingConstants{
    public static final int kLimeLightAngle = 35;
    public static final double kShooterAngle = 62.23;

    public static final Pose2d blueSpeakerPos = new Pose2d(new Translation2d(0.0, 5.55), new Rotation2d());
    public static final Pose2d redSpeakerPos = new Pose2d(new Translation2d(16.5, 5.55), new Rotation2d());
  }

}
