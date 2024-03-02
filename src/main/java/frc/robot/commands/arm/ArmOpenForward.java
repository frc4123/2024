package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ArmOpenVelo;

public class ArmOpenForward extends Command{
    ArmOpenVelo arm;

    public ArmOpenForward(ArmOpenVelo arm){
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.setArmVelo(1);
    }

    @Override
    public void end(boolean interrupted) {
        arm.setArmVelo(0);
    }
}
