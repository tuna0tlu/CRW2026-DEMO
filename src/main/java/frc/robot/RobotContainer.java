// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.drivetrain.SwerveSubsystem;
import frc.robot.drivetrain.commands.TeleopSwerve;
import java.util.Map;
import java.util.Random;

public class RobotContainer {
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final CommandXboxController driverController = new CommandXboxController(0);
    private boolean fieldOriented = true;
    private final Random random = new Random();

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

    private Command createDynamicAutonomous() {
        PathConstraints constraints = new PathConstraints(
            DriveConstants.MAX_SPEED_METERS_PER_SECOND, 
            3.0, 
            DriveConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, 
            Math.PI
        );


        Command blueCoralLeft = AutoBuilder.pathfindToPose(new Pose2d(AutoConstants.BLUE_CORAL_STATION_LEFT, new Rotation2d()), constraints);
        Command blueCoralRight = AutoBuilder.pathfindToPose(new Pose2d(AutoConstants.BLUE_CORAL_STATION_RIGHT, new Rotation2d()), constraints);
        Command redCoralLeft = AutoBuilder.pathfindToPose(new Pose2d(AutoConstants.RED_CORAL_STATION_LEFT, new Rotation2d()), constraints);
        Command redCoralRight = AutoBuilder.pathfindToPose(new Pose2d(AutoConstants.RED_CORAL_STATION_RIGHT, new Rotation2d()), constraints);
        Command blueReefLeft = AutoBuilder.pathfindToPose(new Pose2d(AutoConstants.BLUE_REEF_LEFT, new Rotation2d()), constraints);
        Command blueReefRight = AutoBuilder.pathfindToPose(new Pose2d(AutoConstants.BLUE_REEF_RIGHT, new Rotation2d()), constraints);
        Command redReefLeft = AutoBuilder.pathfindToPose(new Pose2d(AutoConstants.RED_REEF_LEFT, new Rotation2d()), constraints);
        Command redReefRight = AutoBuilder.pathfindToPose(new Pose2d(AutoConstants.RED_REEF_RIGHT, new Rotation2d()), constraints);

        Command coralAndReefLoop = new SequentialCommandGroup(
            // Coral Station'a git
            Commands.select(
                Map.of(
                    0, blueCoralLeft,
                    1, blueCoralRight,
                    2, redCoralLeft,
                    3, redCoralRight
                ),
                () -> {
                    boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
                    return (isRed ? 2 : 0) + (random.nextBoolean() ? 1 : 0);
                }
            ),
            Commands.waitSeconds(2.0),
            // Reef'e git
            Commands.select(
                Map.of(
                    0, blueReefLeft,
                    1, blueReefRight,
                    2, redReefLeft,
                    3, redReefRight
                ),
                () -> {
                    boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
                    return (isRed ? 2 : 0) + (random.nextBoolean() ? 1 : 0);
                }
            ),
            Commands.waitSeconds(2.0)
        ).repeatedly();

        return new SequentialCommandGroup(
            AutoBuilder.buildAuto("A to B"),
            coralAndReefLoop
        );
    }

    public Command getAutonomousCommand() {
        return createDynamicAutonomous();
    }
}