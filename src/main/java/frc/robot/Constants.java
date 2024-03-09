package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {

  public static final class SubsystemConstants {
    public static final int Front_Left_Arm = 10; 
    public static final int Front_Right_Arm = 11;
    public static final int Back_Left_Arm = 12;
    public static final int Back_Right_Arm = 13; // back right

    public static final int Left_Shooter = 16;
    public static final int Right_Shooter = 15;

    public static final int Ground_Intake = 17; 
    
    public static final int Intake_Skipper = 18;

    public static final int Right_Climber = 19; //leader
    public static final int Left_Climber = 20;
  }

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
    // determines if drive motors are reversed3

    public static final boolean Front_Left_Turning_Encoder_Reversed = true;
    public static final boolean Front_Right_Turning_Encoder_Reversed = true;
    public static final boolean Back_Left_Turning_Encoder_Reversed = true;
    public static final boolean Back_Right_Turning_Encoder_Reversed = true;
    // determintes if turn motors are reversed

    public static final double Front_Left_Drive_CANcoder_Offset_Rad = 0; // 2 * math.pi
    public static final double Front_Right_Drive_CANcoder_Offset_Rad = 0; 
    public static final double Back_Left_Drive_CANcoder_Offset_Rad = 0; 
    public static final double Back_Right_Drive_CANcoder_Offset_Rad = 0; 

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
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3; // 3
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3; // 3
  }

  public static final class InputConstants {
    public static final int kDriverControllerPort0 = 0;
    public static final int kDriverControllerPort1 = 1;
    public static final int kDriverControllerPort2 = 2;
    public static final boolean fieldOrientation = true;
    public static final double kDeadband = 0.034123;
  }

  public static final class PIDTuning {
    public static final double Arm_PID_P = 0.0725; // 0.075 was 0.1 
    public static final double Arm_PID_I = 0;
    public static final double Arm_PID_D = 0.0145;
    public static final double Arm_FF_S = 0;
    public static final double Arm_FF_G = 0.25; // start here 
    public static final double Arm_FF_A = 0;
    public static final double Arm_POSITION_PLACE = 40.4123; // 40.642 with bumper // 35 without bumper
    public static final double Arm_POSITION_SAFE = 35.547;
    public static final double Arm_POSITION_SHOOT = 8; //be very careful, slowly increase to 5 and keep going until reaches optimal position
    public static final double Arm_POSITION_INTAKE = 0.2; 
    public static final double Arm_CONSTRAINTS_VELOCITY = 35; // increase value upon having proper values | THIS STEP IS LAST
    public static final double Arm_CONSTRAINTS_ACCELERATION = 70; // increase value upon having proper values | THIS STEP IS LAST
  }

  public static final class VisionConstants {
    public static final double kCameraHeight = 0;
    public static final double kTargetHeight = 0;
    public static final double kCameraPitch = 0;
    public static final double kTargetPitch = 0;
    public static final double targetPose = 0;
    public static final double cameraToRobot = 0;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = DrivingConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
    public static final double kMaxAngularSpeedRadiansPerSecond = DrivingConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3; 
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
    public static final double kPXController = 1.5;
    public static final double kPYController = 1.5;
    public static final double kPThetaController = 3;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = 
      new TrapezoidProfile.Constraints(
      kMaxAngularSpeedRadiansPerSecond,
      kMaxAngularAccelerationRadiansPerSecondSquared);
  }

  public static final class ClimberConstants {
    public static final double upperThreshold = 50;
    public static final double lowerThreshold = -0.00001;
  }

  public static final class ModuleConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4); // wheel diameter
    public static final double kDriveMotorGearRatio = 1 / 6.75; // motor gear ratio
    public static final double TurningMotorGearRatio = 1.0 / (150 / 7); // turning gear ratio
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = TurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
    public static final double kPTurning = 0.375; 
    public static final double kPhysicalMaxSpeedMetersPerSecond = 5; 
    // the above will have to be changed after gear ratio measurements, wheel
    // measurements, and pid tuning

  }
}
