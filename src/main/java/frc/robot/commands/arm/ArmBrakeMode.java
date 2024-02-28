package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ArmBrakeMode extends Command{
    Arm arm;

    public ArmBrakeMode(Arm arm){
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.enableBrakeMode(true);
        System.out.println("brakemodecalled");
    }

    @Override
    public void end(boolean interrupted) {
    }
    
    
    
}
