package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class ElevatorToSetpoint extends CommandBase{
    private double setpoint;
    private Elevator mElevator;

    public ElevatorToSetpoint(Elevator elevator, double s){
        setpoint = s;
        mElevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void execute(){
        mElevator.reachGoal(setpoint);
    }

    @Override
    public void end(boolean interrupted){
        mElevator.setPercentOutput(0);
    }
}
