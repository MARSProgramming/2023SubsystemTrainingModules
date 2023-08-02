package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSimulation extends SubsystemBase implements AutoCloseable{
    private final DCMotor m_elevatorGearbox = DCMotor.getFalcon500(4);

    private final Encoder m_encoder =
        new Encoder(Constants.kEncoderAChannel, Constants.kEncoderBChannel);
    private final PWMSparkMax m_motor = new PWMSparkMax(Constants.kMotorPort);
  
    // Simulation classes help us simulate what's going on, including gravity.
    private final ElevatorSim m_elevatorSim =
        new ElevatorSim(
            m_elevatorGearbox,
            Constants.kElevatorGearing,
            Constants.kCarriageMass,
            Constants.kElevatorDrumRadius,
            Constants.kMinElevatorHeightMeters,
            Constants.kMaxElevatorHeightMeters,
            true,
            VecBuilder.fill(0.01));
    private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);
    private final PWMSim m_motorSim = new PWMSim(m_motor);
  
    // Create a Mechanism2d visualization of the elevator
    private final Mechanism2d m_mech2d = new Mechanism2d(20, 12);
    private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 10, 1);
    private final MechanismLigament2d m_elevatorMech2d =
        m_mech2dRoot.append(
            new MechanismLigament2d("Elevator", m_elevatorSim.getPositionMeters(), 90));
  
    /** Subsystem constructor. */
    public ElevatorSimulation() {
      m_encoder.setDistancePerPulse(Constants.kElevatorEncoderDistPerPulse);
  
      // Publish Mechanism2d to SmartDashboard
      // To view the Elevator visualization, select Network Tables -> SmartDashboard -> Elevator Sim
      SmartDashboard.putData("Elevator Sim", m_mech2d);
    }
  
    /** Advance the simulation. */
    public void simulationPeriodic() {
      // In this method, we update our simulation of what our elevator is doing
      // First, we set our "inputs" (voltages)
      m_elevatorSim.setInput(m_motorSim.getSpeed() * RobotController.getBatteryVoltage());
  
      // Next, we update it. The standard loop time is 20ms.
      m_elevatorSim.update(0.020);
  
      // Finally, we set our simulated encoder's readings and simulated battery voltage
      m_encoderSim.setDistance(m_elevatorSim.getPositionMeters());
      // SimBattery estimates loaded battery voltages
      RoboRioSim.setVInVoltage(
          BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));
    }
  
    /**
     * Run control loop to reach and maintain goal.
     *
     * @param goal the position to maintain
     */
    public void setMotorVoltage(double v) {
      m_motor.setVoltage(v);
    }

    public void setPercentOutput(double v) {
        m_motor.set(v);
    }
    public double getEncoderPosition(){
        return m_encoder.getDistance();
    }
    public double getVelocity(){
        return m_encoder.getRate();
    }
  
    /** Stop the control loop and motor output. */
    public void stop() {
        m_motor.set(0.0);
    }
  
    /** Update telemetry, including the mechanism visualization. */
    public void updateTelemetry() {
      // Update elevator visualization with position
      m_elevatorMech2d.setLength(m_encoder.getDistance());
    }
  
    @Override
    public void close() {
      m_encoder.close();
      m_motor.close();
      m_mech2d.close();
    }

    @Override
    public void periodic(){
        updateTelemetry();
        simulationPeriodic();
    }
}
