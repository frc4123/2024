// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivingConstants;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

public class TaxiLeftBlue extends SubsystemBase {
  SwerveSubsystem swerve;

  /** Creates a new Autos. */
  public TaxiLeftBlue(SwerveSubsystem swerveSubsystem) {
    this.swerve = swerveSubsystem;
    AutoBuilder.configureHolonomic(
        swerve::getPose, // Robot pose supplier
        swerve::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        swerve::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        swerve::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                                         AutoConstants.TRANSLATION_PID,
                                         // Translation PID constants
                                         AutoConstants.ANGLE_PID,
                                         // Rotation PID constants
                                         4.5,
                                         // Max module speed, in m/s
                                         swerve.getSwerveDriveConfiguration().getDriveBaseRadiusMeters(),
                                         // Drive base radius in meters. Distance from robot center to furthest module.
                                         new ReplanningConfig()
                                         // Default path replanning config. See the API for the options here
        ),
        () -> false,
        this // Reference to this subsystem to set requirements
                                  );
  }

  public Command taxiLeftBlue() {
    return AutoBuilder.buildAuto("taxiLeftBlue");
  }

  @Override
  public void periodic() {
  }
}
