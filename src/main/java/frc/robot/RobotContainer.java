// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.drivetrain.SwerveSubsystem;

public class RobotContainer {
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final CommandXboxController driverController = new CommandXboxController(0);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        //TODO: Yarın eklenecek
    }

    public Command getAutonomousCommand() {
        return null;
    }
}