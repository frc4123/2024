package frc.robot.subsystems;

import frc.robot.Constants.DrivingConstants;
import frc.robot.Constants.ModuleConstants;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.TimedRobot;

public class SwerveSubsystem extends SubsystemBase{

    Vision swerveVision;
    private long lastCorrection = 0;
    private final long corectionInterval = 50;

    private final SwerveModule frontLeft = new SwerveModule(
            DrivingConstants.Front_Left_Drive,
            DrivingConstants.Front_Left_Turn,
            DrivingConstants.Front_Left_Drive_Encoder_Reversed,
            DrivingConstants.Front_Left_Turning_Encoder_Reversed,
            DrivingConstants.Front_Left_Drive_CANcoder,
            DrivingConstants.Front_Left_Drive_CANcoder_Offset_Rad,
            DrivingConstants.Front_Left_Drive_Absolute_Encoder_Reversed);

    private final SwerveModule frontRight = new SwerveModule(
            DrivingConstants.Front_Right_Drive,
            DrivingConstants.Front_Right_Turn,
            DrivingConstants.Front_Right_Drive_Encoder_Reversed,
            DrivingConstants.Front_Right_Turning_Encoder_Reversed,
            DrivingConstants.Front_Right_Drive_CANcoder,
            DrivingConstants.Front_Right_Drive_CANcoder_Offset_Rad,
            DrivingConstants.Front_Right_Drive_Absolute_Encoder_Reversed);

    private final SwerveModule backLeft = new SwerveModule(
            DrivingConstants.Back_Left_Drive,
            DrivingConstants.Back_Left_Turn,
            DrivingConstants.Back_Left_Drive_Encoder_Reversed,
            DrivingConstants.Back_Left_Turning_Encoder_Reversed,
            DrivingConstants.Back_Left_Drive_CANcoder,
            DrivingConstants.Back_Left_Drive_CANcoder_Offset_Rad,
            DrivingConstants.Back_Left_Drive_Absolute_Encoder_Reversed);

    private final SwerveModule backRight = new SwerveModule(
            DrivingConstants.Back_Right_Drive,
            DrivingConstants.Back_Right_Turn,
            DrivingConstants.Back_Right_Drive_Encoder_Reversed,
            DrivingConstants.Back_Right_Turning_Encoder_Reversed,
            DrivingConstants.Back_Right_Drive_CANcoder,
            DrivingConstants.Back_Right_Drive_CANcoder_Offset_Rad,
            DrivingConstants.Back_Right_Drive_Absolute_Encoder_Reversed);

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DrivingConstants.kDriveKinematics,
        new Rotation2d(), 
        new SwerveModulePosition[]{
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        }
        );

        public SwerveSubsystem() {
            new Thread(() -> {
                try {
                    Thread.sleep(1000);
                    zeroHeading();
                } catch (Exception e) {}
            }).start();
        }
        // creates new thread to reset gyro

    public void zeroHeading() {
        gyro.reset();
        gyro.setAngleAdjustment(0);
    }

    public double getHeading() {
        return Math.IEEEremainder(-gyro.getAngle(), 360);
        // return 0;
    }
    // keeps degree within 360 degree measurements

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Rotation2d getRotation2dFromOdometry() {
        return odometer.getPoseMeters().getRotation();
    }
    // wpilib 

    public Pose2d getPose() {
        return odometer.getPoseMeters();   
    }

    // converts vision Pose3d to Pose2d 
    public Pose2d getVisionPose() {
        Pose3d visionPose3d = swerveVision.get3dPose();
        Pose2d convertedPose2d = visionPose3d.toPose2d();

        return convertedPose2d;
    }

    public void overrideSwervePose() {
        double poseAngleDifference = getVisionPose().getRotation().getDegrees() - getHeading();
        gyro.setAngleAdjustment(poseAngleDifference);

        odometer.update(
            getVisionPose().getRotation(),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            });

    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        }, pose);
    }

    @Override
    public void periodic() {
        odometer.update(
            gyro.getRotation2d(),
            new SwerveModulePosition[] {
              frontLeft.getPosition(),
              frontRight.getPosition(),
              backLeft.getPosition(),
              backRight.getPosition()
            });

        long currentTime = System.currentTimeMillis();

        if (currentTime - lastCorrection >= corectionInterval && swerveVision.hasTarget()) {
            overrideSwervePose();
            lastCorrection = currentTime;
        }

        SmartDashboard.putNumber("Robot Heading", getHeading());
        //SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());

        SmartDashboard.putString("Front Left State", frontLeft.getState().toString());
        SmartDashboard.putString("Front Right State", frontRight.getState().toString());
        SmartDashboard.putString("Back Left State", backLeft.getState().toString());
        SmartDashboard.putString("Back Right State", backRight.getState().toString());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, ModuleConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
    //sets wheel speeds at same speed (desaturate)
}
