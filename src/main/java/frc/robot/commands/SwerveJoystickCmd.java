package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCmd extends Command {
    
    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
        Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
        Supplier<Boolean> fieldOrientedFunction) {
            this.swerveSubsystem = swerveSubsystem;
            this.xSpdFunction = xSpdFunction;
            this.ySpdFunction = ySpdFunction;
            this.turningSpdFunction = turningSpdFunction;
            this.fieldOrientedFunction = fieldOrientedFunction;
            this.xLimiter = new SlewRateLimiter(0); // Input Constants
            this.yLimiter = new SlewRateLimiter(0); // Input Constants
            this.turningLimiter = new SlewRateLimiter(0); // Input Constants
            addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        // 1. Get real-time joystick inputs
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = turningSpdFunction.get();

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > 0.01 ? xSpeed  : 0;
        ySpeed = Math.abs(ySpeed) > 0.01 ? ySpeed  : 0;
        turningSpeed = Math.abs(turningSpeed) > 0.01 ? turningSpeed  : 0;

        // 3. Make the driving smoother
        xSpeed = xLimiter.calculate(xSpeed); // Multiply by max speed
        ySpeed = yLimiter.calculate(ySpeed); // Multiply by max speed
        turningSpeed = turningLimiter.calculate(turningSpeed); // Multiply by max speed

        // 4. Construct desired chassis speeds
        @SuppressWarnings("unused")
        ChassisSpeeds chassisSpeeds;
        if (fieldOrientedFunction.get()) {
            // field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
        } else {
            // robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        // 5. Convert chassis speeds to individual module states
        // SwerveModuleState[] moduleStates = new SwerveModuleState[]; // TODO use kinematics method from constants  toModuleStates

        // 6. Output each module states to wheels
        // swerveSubsystem.setModuleStates(/* moduleStates */); TODO
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
