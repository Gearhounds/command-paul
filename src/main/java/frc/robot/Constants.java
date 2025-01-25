package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {
    
    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = (4/27);
        public static final double kTurningMotorGearRatio = (7/150);
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.005;
        public static final double kITurning = 0.002;
    }

    public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(22.75);
        public static final double kWheelBase = Units.inchesToMeters(24.75);

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2));
     

            

        public static final double kMaxSpeedMetersPerSecond = 4.6;
        public static final double kMaxTurnSpeedRadPerSecond = 2 * 2 * Math.PI; // TODO We guessed

        // FRONT LEFT
        public static final int kFrontLeftDriveMotorPort = 2;
        public static final int kFrontLeftTurningMotorPort = 1;
        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final int kFrontLeftAbsEncoderPort = 9;
        public static final double kFrontLeftAbsEncoderOffsetRad = 0;
        public static final boolean kFrontLeftAbsEncoderReversed = false;

        

        // FRONT RIGHT
        public static final int kFrontRightDriveMotorPort = 4;
        public static final int kFrontRightTurningMotorPort = 3;
        public static final boolean kFrontRightDriveEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = false;
        public static final int kFrontRightAbsEncoderPort = 12;
        public static final double kFrontRightAbsEncoderOffsetRad = 0;
        public static final boolean kFrontRightAbsEncoderReversed = false;

        // BACK LEFT
        public static final int kBackLeftDriveMotorPort = 8;
        public static final int kBackLeftTurningMotorPort = 7;
        public static final boolean kBackLeftDriveEncoderReversed = false;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final int kBackLeftAbsEncoderPort = 10;
        public static final double kBackLeftAbsEncoderOffsetRad = 0;
        public static final boolean kBackLeftAbsEncoderReversed = false;

        // BACK RIGHT
        public static final int kBackRightDriveMotorPort = 6;
        public static final int kBackRightTurningMotorPort = 5;
        public static final boolean kBackRightDriveEncoderReversed = false;
        public static final boolean kBackRightTurningEncoderReversed = true;
        public static final int kBackRightAbsEncoderPort = 11;
        public static final double kBackRightAbsEncoderOffsetRad = 0;
        public static final boolean kBackRightAbsEncoderReversed = false;
    }

}
