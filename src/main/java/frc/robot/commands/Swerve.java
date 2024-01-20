package frc.robot.commands;

import java.util.function.Supplier;

import frc.robot.Constants.InputConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.DrivingConstants;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;



public class Swerve extends Command{

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    public Swerve(SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.xLimiter = new SlewRateLimiter(DrivingConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DrivingConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DrivingConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override 
    public void execute() {

        // 1. Get real-time joystick inputs
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = turningSpdFunction.get();

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > InputConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > InputConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > InputConstants.kDeadband ? turningSpeed : 0.0;

     SmartDashboard.putString("swerve inputs", String.format(
        "xSpeed: %.2f\tySpeed: %.2f\tturningSpeed: %.2f", xSpeed, ySpeed, turningSpeed));

        // 3. Make the driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * DrivingConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DrivingConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed) * DrivingConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
       
        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        if (InputConstants.fieldOrientation) {
            // field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
        } else {
            // robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DrivingConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);
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
