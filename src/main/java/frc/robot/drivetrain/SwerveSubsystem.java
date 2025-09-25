package frc.robot.drivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

public class SwerveSubsystem extends SubsystemBase {

    private final SwerveModule[] modules;
    private final SwerveDriveKinematics kinematics;
    private final AHRS gyro;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Vision vision;
    private final Field2d field = new Field2d();

    public SwerveSubsystem() {
        gyro = new AHRS(NavXComType.kMXP_SPI);
        vision = new Vision();

        modules = new SwerveModule[] {
            new SwerveModule(SwerveConstants.FL_DRIVE_ID, SwerveConstants.FL_ANGLE_ID, SwerveConstants.FL_ENCODER_ID, SwerveConstants.FL_OFFSET),
            new SwerveModule(SwerveConstants.FR_DRIVE_ID, SwerveConstants.FR_ANGLE_ID, SwerveConstants.FR_ENCODER_ID, SwerveConstants.FR_OFFSET),
            new SwerveModule(SwerveConstants.BL_DRIVE_ID, SwerveConstants.BL_ANGLE_ID, SwerveConstants.BL_ENCODER_ID, SwerveConstants.BL_OFFSET),
            new SwerveModule(SwerveConstants.BR_DRIVE_ID, SwerveConstants.BR_ANGLE_ID, SwerveConstants.BR_ENCODER_ID, SwerveConstants.BR_OFFSET)
        };
        
        Translation2d fl = new Translation2d(SwerveConstants.CHASSIS_LENGTH_METERS / 2, SwerveConstants.CHASSIS_WIDTH_METERS / 2);
        Translation2d fr = new Translation2d(SwerveConstants.CHASSIS_LENGTH_METERS / 2, -SwerveConstants.CHASSIS_WIDTH_METERS / 2);
        Translation2d bl = new Translation2d(-SwerveConstants.CHASSIS_LENGTH_METERS / 2, SwerveConstants.CHASSIS_WIDTH_METERS / 2);
        Translation2d br = new Translation2d(-SwerveConstants.CHASSIS_LENGTH_METERS / 2, -SwerveConstants.CHASSIS_WIDTH_METERS / 2);

        kinematics = new SwerveDriveKinematics(fl, fr, bl, br);
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, gyro.getRotation2d(), getModulePositions(), new Pose2d());

        
        RobotConfig ppConfig;
        try {
            ppConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            throw new RuntimeException("PathPlanner RobotConfig GUI ayarları yüklenemedi", e);
        }

        AutoBuilder.configure(
            this::getPose, 
            this::resetOdometry, 
            this::getRobotRelativeSpeeds,
            this::driveRobotRelative,
            new PPHolonomicDriveController(
                Constants.AutoConstants.TRANSLATION_PID, 
                Constants.AutoConstants.ANGLE_PID
            ),
            ppConfig,
            () -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red,
            this
        );

        SmartDashboard.putData("Field", field);
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.DriveConstants.MAX_SPEED_METERS_PER_SECOND);
        for (int i = 0; i < modules.length; i++) {
            modules[i].setDesiredState(moduleStates[i]);
        }
    }

    private void driveRobotRelative(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {
        drive(speeds);
    }
    
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
    }
    
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[]{
            modules[0].getPosition(), modules[1].getPosition(),
            modules[2].getPosition(), modules[3].getPosition()
        };
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[]{
            modules[0].getState(), modules[1].getState(),
            modules[2].getState(), modules[3].getState()
        };
    }

    @Override
    public void periodic() {
        poseEstimator.update(gyro.getRotation2d(), getModulePositions());
        vision.getEstimatedRobotPose().ifPresent(est ->
            poseEstimator.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds)
        );
        field.setRobotPose(getPose());
    }
}