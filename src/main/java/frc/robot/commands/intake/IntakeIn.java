package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Intake;

public class IntakeIn extends Command{
    Intake intake;

    public IntakeIn(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.setIntakeVelo(-0.7); 
    }

    @Override
    public void end(boolean interrupted) {
        intake.setIntakeVelo(0.0); 
    }
    
}
