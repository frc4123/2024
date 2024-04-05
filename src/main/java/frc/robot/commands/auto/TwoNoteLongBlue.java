// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import frc.robot.Constants.DrivingConstants;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

public class TwoNoteLongBlue extends SubsystemBase {
  SwerveSubsystem swerve;

  /** Creates a new Autos. */
  public TwoNoteLongBlue(SwerveSubsystem swerveSubsystem) {
    this.swerve = swerveSubsystem;
    AutoBuilder.configureHolonomic(
        () -> swerve.getPose(),
        (pose2d) -> swerve.resetOdometry(pose2d),
        swerve::getCurrentRobotChassiSpeeds,
        (speeds) -> swerve.setRobotChassiSpeeds(speeds),
         new HolonomicPathFollowerConfig(new PIDConstants(3, .01, 0), new PIDConstants(1.7, 0.06, 0), 4, DrivingConstants.kWheelBase / 2, new ReplanningConfig()),
        () -> false, 
        swerveSubsystem
        );
  }

  public Command twoNoteLongBlue() {
    return AutoBuilder.buildAuto("twoNoteBlue");
  }

  @Override
  public void periodic() {
  }
}
