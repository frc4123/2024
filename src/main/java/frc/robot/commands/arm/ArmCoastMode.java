package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ArmCoastMode extends Command{
    Arm arm;

    public ArmCoastMode(Arm arm){
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.enableCoastMode(true);
        System.out.println("coastmodecalled");
    }

    @Override
    public void end(boolean interrupted) {
    }
    
    
    
}
