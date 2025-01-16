package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;

public class SwerveModule {

    public final SparkMax driveMotor;
    public final SparkMax turningMotor;

    SparkMaxConfig driveConfig;
    SparkMaxConfig turnConfig;
    

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;

    private final SparkAbsoluteEncoder absEncoder;
    private final boolean absEncoderReversed;
    private final double absEncoderOffsetRad;

    public SwerveModule (int driveMotorID, int turningMotorId, boolean driveMotorReversed,
        boolean turningMotorReversed, int absEncoderID, double absEncoderOffset, boolean absEncoderReversed) {

            driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
            turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);
            
            this.absEncoderOffsetRad = absEncoderOffset;
            this.absEncoderReversed = absEncoderReversed;
            absEncoder = turningMotor.getAbsoluteEncoder();

            driveConfig = new SparkMaxConfig();
            driveConfig
                .inverted(driveMotorReversed)
                .idleMode(IdleMode.kBrake);

            turnConfig = new SparkMaxConfig();
            turnConfig
                .inverted(turningMotorReversed)
                .idleMode(IdleMode.kBrake);
            
            driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
            turningMotor.configure(turnConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

            driveEncoder = driveMotor.getEncoder();
            turningEncoder = turningMotor.getEncoder();

            turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
            turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        }

        public double getDrivePosition() {
            return driveEncoder.getPosition();
        }

        public double getTurningPosition() {
            return turningEncoder.getPosition();
        }

        public double getDriveVelocity() {
            return driveEncoder.getVelocity();
        }

        public double getTurningVelocity() {
            return turningEncoder.getVelocity();
        }

        public double getAbsEncoderRad() {
            double angle = absEncoder.getPosition();
            angle *= 2 * Math.PI;
            angle -= absEncoderOffsetRad;
            return angle * (absEncoderReversed ? -1 : 1);
        }

        public void resetEncoders() {
            driveEncoder.setPosition(0);
            turningEncoder.setPosition(getAbsEncoderRad());
        }
        
        public SwerveModuleState getState() {
            return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
        }
        
        public void setDesiredState(SwerveModuleState state) {
            if (Math.abs(state.speedMetersPerSecond) < 0.001) {
                stop();
                return;
            }
            state = SwerveModuleState.optimize(state, getState().angle);
            driveMotor.set(state.speedMetersPerSecond / Constants.DriveConstants.kMaxSpeedMetersPerSecond);
            turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
            
        }

        public void stop() {
            driveMotor.set(0);
            turningMotor.set(0);
        }
}
