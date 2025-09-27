// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final class VisionConstants {
        public static final String CAMERA_NAME = "photonvision"; 
        public static final Transform3d ROBOT_TO_CAM = 
            new Transform3d(new Translation3d(0.35, 0.0, 0.15), new Rotation3d(0, 0, 0));
    }

    public static final class AutoConstants {
        public static final PIDConstants TRANSLATION_PID = new PIDConstants(5.0, 0.0, 0.0);
        public static final PIDConstants ANGLE_PID = new PIDConstants(5.0, 0.0, 0.0);
        public static final Translation2d FIELD_CENTER = new Translation2d(8.27, 4.11);
        public static final Translation2d BLUE_CORAL_STATION_LEFT = new Translation2d(1.5, 5.5);
        public static final Translation2d BLUE_CORAL_STATION_RIGHT = new Translation2d(1.5, 2.5);
        public static final Translation2d RED_CORAL_STATION_LEFT = new Translation2d(15.0, 5.5);
        public static final Translation2d RED_CORAL_STATION_RIGHT = new Translation2d(15.0, 2.5);
        public static final Translation2d BLUE_REEF_LEFT = new Translation2d(4.0, 6.5);
        public static final Translation2d BLUE_REEF_RIGHT = new Translation2d(4.0, 1.5);
        public static final Translation2d RED_REEF_LEFT = new Translation2d(12.5, 6.5);
        public static final Translation2d RED_REEF_RIGHT = new Translation2d(12.5, 1.5);
    }

    public static final class DriveConstants {
        public static final double MAX_SPEED_METERS_PER_SECOND = 4.5;
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 2 * Math.PI;
        public static final double JOYSTICK_DEADBAND = 0.1;
    
        public static final double TRANSLATION_ACCEL_LIMIT = 3.0; 
        public static final double ROTATION_ACCEL_LIMIT = 3.0;
    }

    public static final class SwerveConstants {
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);
        public static final double DRIVE_GEAR_RATIO = 6.5;
        public static final double ANGLE_GEAR_RATIO = 11.7187;
        public static final double DRIVE_ENCODER_CONVERSION_FACTOR = (WHEEL_DIAMETER_METERS * Math.PI) / DRIVE_GEAR_RATIO;
        public static final double ANGLE_ENCODER_CONVERSION_FACTOR = (2 * Math.PI) / ANGLE_GEAR_RATIO;
        public static final double CHASSIS_LENGTH_METERS = 0.74;
        public static final double CHASSIS_WIDTH_METERS = 0.74;
        public static final double ANGLE_KP = 0.8;
        public static final double DRIVE_KP = 0.1;
        public static final double DRIVE_KFF = 0.3;
        public static final int FL_DRIVE_ID = 2, FL_ANGLE_ID = 1, FL_ENCODER_ID = 0;
        public static final int FR_DRIVE_ID = 4, FR_ANGLE_ID = 3, FR_ENCODER_ID = 2;
        public static final int BL_DRIVE_ID = 6, BL_ANGLE_ID = 5, BL_ENCODER_ID = 1;
        public static final int BR_DRIVE_ID = 9, BR_ANGLE_ID = 7, BR_ENCODER_ID = 3;
        public static final double FL_OFFSET = 194 + 180.0;
        public static final double FR_OFFSET = 146.5 + 180.0;
        public static final double BL_OFFSET = 356.3 + 180.0;
        public static final double BR_OFFSET = 245.5 + 180.0;
    }

}