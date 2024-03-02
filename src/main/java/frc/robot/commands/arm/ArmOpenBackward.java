package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ArmOpenVelo;

public class ArmOpenBackward extends Command{
    ArmOpenVelo arm;

    public ArmOpenBackward(ArmOpenVelo arm){
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.setArmVelo(-0.4);
    }

    @Override
    public void end(boolean interrupted) {
        arm.setArmVelo(0);
    }
}
