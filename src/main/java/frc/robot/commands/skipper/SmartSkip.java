package frc.robot.commands.skipper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PIDTuning;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Skipper;
import frc.robot.subsystems.Shooter;

public class SmartSkip extends Command {
    Skipper skipper;
    Arm arm;
    Shooter shooter;

    public SmartSkip(Skipper skipper, Arm arm, Shooter shooter) {
        this.skipper = skipper;
        this.arm = arm;
        this.shooter = shooter;
        addRequirements(skipper);
    }

    @Override
    public void execute() {
        skipper.setSkipperVelo(0.8);
    }

    @Override
    public void end(boolean interrupted) {
        skipper.setSkipperVelo(0.0);
    }

    public void periodic() {
        if (arm.getArmPosition() < PIDTuning.Arm_Position_Threshold_UP &&
            arm.getArmPosition() > PIDTuning.Arm_Position_Threshold_Down &&
            shooter.getShooterVelo() > PIDTuning.Shooter_Velo_Threshold) {
                
            execute();
        } else {
            end(true);
        }

    }
}
