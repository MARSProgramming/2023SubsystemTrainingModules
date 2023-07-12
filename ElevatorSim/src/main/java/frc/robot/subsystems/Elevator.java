package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator extends ElevatorSimulation{
    private PIDController pid = new PIDController(3, 0, 0);
    public Elevator(){

    }
    public void setPosition(double pos){
        setPercentOutput(pid.calculate(getEncoderPosition(), pos));
        SmartDashboard.putNumber("Goal", pos);
        SmartDashboard.putNumber("Actual", getEncoderPosition());
    }
}