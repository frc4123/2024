package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ArmInitialize extends Command{
    Arm arm;

    public ArmInitialize(Arm arm){
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.setPosition(arm.getPosition());
        // ONLY CALL DURING AUTO INIT
    }

    @Override
    public void end(boolean interrupted) {
    }
}
