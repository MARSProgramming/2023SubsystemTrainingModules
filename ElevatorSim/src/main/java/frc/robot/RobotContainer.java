// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ElevatorToSetpoint;
import frc.robot.subsystems.Elevator;

public class RobotContainer {
    private final CommandXboxController mXboxController = new CommandXboxController(0);
    private final Elevator m_elevator = new Elevator();

    public Elevator getElevator(){return m_elevator;}

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        mXboxController.a().whileTrue(new ElevatorToSetpoint(m_elevator, 7));
        mXboxController.b().whileTrue(new ElevatorToSetpoint(m_elevator, 0));
    }

    public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
    }
}
