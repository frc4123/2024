package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Climber;

public class RightClimbDown extends Command{
    Climber climb;

    public RightClimbDown(Climber climb) {
        this.climb = climb;
        addRequirements(climb);
    }

    @Override
    public void execute(){
        climb.setRightClimberVelo(0.8);
    }

    @Override
    public void end(boolean interrupted) {
        climb.setRightClimberVelo(0.0); 
    }

    public void periodic () {
        if (climb.isUnsafeVelocityDown()) {
            end(true);
            System.out.println("ClimbDown Stopped");
        }
    }
}
