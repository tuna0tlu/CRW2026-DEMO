package frc.robot.drivetrain.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.drivetrain.SwerveSubsystem;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopSwerve extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final DoubleSupplier translationSup;
    private final DoubleSupplier strafeSup;
    private final DoubleSupplier rotationSup;
    private final BooleanSupplier fieldOrientedSup;
    private final BooleanSupplier pointToCenterSup;

    private final PIDController angleController = new PIDController(Constants.AutoConstants.ANGLE_PID.kP, 0, 0);
    private Optional<Rotation2d> targetRotation = Optional.empty();


    private final SlewRateLimiter xLimiter = new SlewRateLimiter(Constants.DriveConstants.TRANSLATION_ACCEL_LIMIT);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(Constants.DriveConstants.TRANSLATION_ACCEL_LIMIT);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(Constants.DriveConstants.ROTATION_ACCEL_LIMIT);

    public TeleopSwerve(SwerveSubsystem swerveSubsystem, DoubleSupplier translationSup, DoubleSupplier strafeSup,  DoubleSupplier rotationSup, BooleanSupplier fieldOrientedSup, BooleanSupplier pointToCenterSup) {
        this.swerveSubsystem = swerveSubsystem;
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.fieldOrientedSup = fieldOrientedSup;
        this.pointToCenterSup = pointToCenterSup;

        angleController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(swerveSubsystem);
    }


    @Override
    public void execute(){
        
        double rawTranslationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.DriveConstants.JOYSTICK_DEADBAND);
        double rawStrafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.DriveConstants.JOYSTICK_DEADBAND);
        double rawRotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.DriveConstants.JOYSTICK_DEADBAND);

        
        double targetXSpeed = Math.pow(rawTranslationVal, 3) * Constants.DriveConstants.MAX_SPEED_METERS_PER_SECOND;
        double targetYSpeed = Math.pow(rawStrafeVal, 3) * Constants.DriveConstants.MAX_SPEED_METERS_PER_SECOND;
        
        
        double translationVal = xLimiter.calculate(targetXSpeed);
        double strafeVal = yLimiter.calculate(targetYSpeed);

        double rotationOutput;
        if (targetRotation.isPresent()) {
            // Butonla tetiklenen özel bir hedef açıya yönel
            angleController.setSetpoint(targetRotation.get().getRadians());
            rotationOutput = angleController.calculate(swerveSubsystem.getPose().getRotation().getRadians());

            if (angleController.atSetpoint()) {

                targetRotation = Optional.empty();

            }
        } else if (pointToCenterSup.getAsBoolean()) {
            // Buton basılıyken sahanın merkezine yönel
            Rotation2d angleToCenter = Constants.AutoConstants.FIELD_CENTER
                .minus(swerveSubsystem.getPose().getTranslation()).getAngle();
            angleController.setSetpoint(angleToCenter.getRadians());
            rotationOutput = angleController.calculate(swerveSubsystem.getPose().getRotation().getRadians());

        } else {
            // Normal joystick ile dönüş kontrolü
            double targetRotSpeed = Math.pow(rawRotationVal, 3) * Constants.DriveConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;
            rotationOutput = rotLimiter.calculate(targetRotSpeed);
            
        }


        ChassisSpeeds chassisSpeeds = fieldOrientedSup.getAsBoolean()
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translationVal,
                strafeVal,
                rotationOutput,
                swerveSubsystem.getPose().getRotation()
            )
            : new ChassisSpeeds(
                translationVal,
                strafeVal,
                rotationOutput
            );

        swerveSubsystem.drive(chassisSpeeds);
    }

    public void setTargetRotation(Rotation2d angle){
        this.targetRotation = Optional.of(angle);
    }
}