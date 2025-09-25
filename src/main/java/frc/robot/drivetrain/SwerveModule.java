package frc.robot.drivetrain;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
    
    private final SparkMax driveMotor;
    private final SparkMax angleMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder angleEncoder;
    private final AnalogEncoder absoluteEncoder;

    private final SparkClosedLoopController angleController;
    private final SparkClosedLoopController driveController;

    private final double absoluteEncoderOffsetDeg;

    public SwerveModule(int driveMotorId, int angleMotorId, int absoluteEncoderId, double absoluteEncoderOffsetDeg) {

        this.driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
        this.angleMotor = new SparkMax(angleMotorId, MotorType.kBrushless);

        this.driveEncoder = driveMotor.getEncoder();
        this.angleEncoder = angleMotor.getEncoder();
        this.absoluteEncoder = new AnalogEncoder(absoluteEncoderId);


        this.absoluteEncoderOffsetDeg = absoluteEncoderOffsetDeg;


        SparkMaxConfig driveConfig = new SparkMaxConfig();
        driveConfig.idleMode(IdleMode.kBrake);
        driveConfig.encoder
            .positionConversionFactor(SwerveConstants.DRIVE_ENCODER_CONVERSION_FACTOR)
            .velocityConversionFactor(SwerveConstants.DRIVE_ENCODER_CONVERSION_FACTOR / 60.0);
        driveConfig.closedLoop.pidf(SwerveConstants.DRIVE_KP, 0.0, 0.0, SwerveConstants.DRIVE_KFF);
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        SparkMaxConfig angleConfig = new SparkMaxConfig();
        angleConfig.idleMode(IdleMode.kBrake);
        angleConfig.encoder
            .positionConversionFactor(SwerveConstants.ANGLE_ENCODER_CONVERSION_FACTOR)
            .velocityConversionFactor(SwerveConstants.ANGLE_ENCODER_CONVERSION_FACTOR / 60.0);
        angleConfig.closedLoop.pid(SwerveConstants.ANGLE_KP, 0.0, 0.0);
        angleConfig.inverted(true);
        angleMotor.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        this.driveController = driveMotor.getClosedLoopController();
        this.angleController = angleMotor.getClosedLoopController();

        
        resetToAbsolute();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            driveEncoder.getVelocity(),
            new Rotation2d(angleEncoder.getPosition())
        );
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveEncoder.getPosition(),
            new Rotation2d(angleEncoder.getPosition())
        );
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState optimized =
            SwerveModuleState.optimize(desiredState, new Rotation2d(angleEncoder.getPosition()));
        driveController.setReference(optimized.speedMetersPerSecond, ControlType.kVelocity);
        angleController.setReference(optimized.angle.getRadians(), ControlType.kPosition);
    }

    public void resetToAbsolute() {
        double absoluteRadians =
            (absoluteEncoder.get() * 2.0 * Math.PI) - Math.toRadians(absoluteEncoderOffsetDeg);
        angleEncoder.setPosition(absoluteRadians);
    }
}