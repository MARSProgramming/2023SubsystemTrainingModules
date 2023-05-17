package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimulatedMotorSubsystem extends SubsystemBase {

    private WPI_TalonFX motor;
    public SimulatedMotorSubsystem() {
        motor = new WPI_TalonFX(0);

        //ignore this
        simulationInit();
    }

    @Override
    public void periodic() {
        
        //ignore
        simPeriodic();
    }


    //SIMULATION CODE DO NOT TOUCH
    TalonFXSimCollection fxSim = motor.getSimCollection();

    Mechanism2d mech = new Mechanism2d(20, 20);
    MechanismRoot2d root = mech.getRoot("root", 10, 10);
    MechanismLigament2d ligament2d = new MechanismLigament2d("motor", 5, 0);
    SingleJointedArmSim armSim = new SingleJointedArmSim(DCMotor.getFalcon500(1), 0, 0, 0, 0, 0, false);

    private void simulationInit(){
        root.append(ligament2d);
    }

    private void simPeriodic(){
        
    }


}
