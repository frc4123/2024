package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {

  public static final class DrivingConstants {

    public static final int Front_Left_Drive = 2;
    public static final int Front_Right_Drive = 3;
    public static final int Back_Left_Drive = 4;
    public static final int Back_Right_Drive = 5;

    // drive motors - order - start top left in clockwise rotation

    public static final int Front_Left_Turn = 6;
    public static final int Front_Right_Turn = 7;
    public static final int Back_Left_Turn = 8;
    public static final int Back_Right_Turn = 9;
    // turn motors - order - start top left in clockwise rotation

    public static final int Front_Left_Drive_CANcoder = 21;
    public static final int Front_Right_Drive_CANcoder = 22;
    public static final int Back_Left_Drive_CANcoder = 23;
    public static final int Back_Right_Drive_CANcoder = 24;
    // assigns absolute encoders

    public static final boolean Front_Left_Drive_Encoder_Reversed = false;
    public static final boolean Front_Right_Drive_Encoder_Reversed = true;
    public static final boolean Back_Left_Drive_Encoder_Reversed = false;
    public static final boolean Back_Right_Drive_Encoder_Reversed = true;
    // determines if drive motors are reversed

    public static final boolean Front_Left_Turning_Encoder_Reversed = true;
    public static final boolean Front_Right_Turning_Encoder_Reversed = true;
    public static final boolean Back_Left_Turning_Encoder_Reversed = true;
    public static final boolean Back_Right_Turning_Encoder_Reversed = true;
    // determintes if turn motors are reversed

    public static final double Front_Left_Drive_CANcoder_Offset_Rad = 0; // 0.54609716 rot * 2 * math.pi
    public static final double Front_Right_Drive_CANcoder_Offset_Rad = 0; // -3.0817674
    public static final double Back_Left_Drive_CANcoder_Offset_Rad = 0; // -1.05077683
    public static final double Back_Right_Drive_CANcoder_Offset_Rad = 0; // -0.80687389
    // encoder offset in radians
    // TO CHANGE FOR OUR ROBOT

    public static final boolean Front_Left_Drive_Absolute_Encoder_Reversed = true;
    public static final boolean Front_Right_Drive_Absolute_Encoder_Reversed = true;
    public static final boolean Back_Left_Drive_Absolute_Encoder_Reversed = true;
    public static final boolean Back_Right_Drive_Absolute_Encoder_Reversed = true;
    // drive encoders not reversed

    public static final double kTrackWidth = Units.inchesToMeters(20.75);
    // Distance between right and left wheels
    public static final double kWheelBase = Units.inchesToMeters(20.75);
    // Distance between front and back wheels

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2), // front left
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // front right
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // back left
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); // back right

    public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    // the above is to be adjusted if need be
  }

  public static final class InputConstants {

    public static final int kDriverControllerPort = 0;
    public static final int kDriverControllerPort2 = 1;
    public static final boolean fieldOrientation = true;
    public static final double kDeadband = 0.05;

  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = DrivingConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
    public static final double kMaxAngularSpeedRadiansPerSecond = //
        DrivingConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
    public static final double kPXController = 1.5;
    public static final double kPYController = 1.5;
    public static final double kPThetaController = 3;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond,
            kMaxAngularAccelerationRadiansPerSecondSquared);
  }

  public static final class ModuleConstants {

    public static final double kWheelDiameterMeters = Units.inchesToMeters(4); // wheel diameter
    public static final double kDriveMotorGearRatio = 1 / 6.75; // motor gear ratio
    public static final double TurningMotorGearRatio = 1.0 / (150 / 7); // turning gear ratio
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = TurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
    public static final double kPTurning = 0.3; //
    public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
    // the above will have to be changed after gear ratio measurements, wheel
    // measurements, and pid tuning

  }

  public static final class PhysicalConstants {

  }

  public static class OperatorConstants {

  }

}
