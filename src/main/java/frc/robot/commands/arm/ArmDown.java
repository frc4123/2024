package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmOpen;

public class ArmDown extends Command{

    ArmOpen arm;

    public ArmDown(ArmOpen arm) {
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.setArmVelo(-0.5); 
    }
    
    @Override
    public void end(boolean interrupted) {
        arm.setArmVelo(0); 
    }
}
