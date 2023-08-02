package frc.robot.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase{
    private PIDController pid = new PIDController(3, 0, 0);
    ElevatorFeedforward feedforward = new ElevatorFeedforward(0, 0.2, 0.7, 0);
    private ElevatorSimulation elevatorSimulation = new ElevatorSimulation();
    public Elevator(){
        
    }
    public void setPercentOutput(double p){
        elevatorSimulation.setPercentOutput(p);
    }
    public void setPosition(double pos){
        elevatorSimulation.setPercentOutput(pid.calculate(elevatorSimulation.getEncoderPosition(), pos) + feedforward.calculate(elevatorSimulation.getVelocity()));
        SmartDashboard.putNumber("Goal", pos);
        SmartDashboard.putNumber("Actual", elevatorSimulation.getEncoderPosition());
    }
    public boolean atSetpoint(double pos){
        return Math.abs(elevatorSimulation.getEncoderPosition() - pos) < 0.1;
    }
}