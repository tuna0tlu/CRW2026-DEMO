// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.drivetrain.SwerveSubsystem;
import frc.robot.drivetrain.commands.TeleopSwerve;

public class RobotContainer {
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final CommandXboxController driverController = new CommandXboxController(0);
    private boolean fieldOriented = true;

    private final TeleopSwerve teleopSwerve;

    public RobotContainer() {
        teleopSwerve = new TeleopSwerve(
            swerveSubsystem,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX(),
            () -> fieldOriented,
            driverController.a()::getAsBoolean
        );
        swerveSubsystem.setDefaultCommand(teleopSwerve);

        configureBindings();
    }

    private void configureBindings() {
        driverController.start().onTrue(Commands.runOnce(() -> fieldOriented = !fieldOriented));

        driverController.y().onTrue(Commands.runOnce(() -> 
            teleopSwerve.setTargetRotation(
                swerveSubsystem.getPose().getRotation().plus(Rotation2d.fromDegrees(180))
            )
        ));
    }

    public Command getAutonomousCommand() {
        return null; //TODO: Otonom komutları
    }
}