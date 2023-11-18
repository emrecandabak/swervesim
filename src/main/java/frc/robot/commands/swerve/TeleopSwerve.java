package frc.robot.commands.swerve;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {    
    private Drive mDrive;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private final SlewRateLimiter xLimiter, yLimiter, rotationLimiter;
    
    public TeleopSwerve(Drive drive, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        mDrive = drive;
        addRequirements(mDrive);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        xLimiter = new SlewRateLimiter(DriveConstants.kMaxAccelerationMetersPerSecondSquared);
        yLimiter = new SlewRateLimiter(DriveConstants.kMaxAccelerationMetersPerSecondSquared);
        rotationLimiter = new SlewRateLimiter(DriveConstants.kMaxAngularAccelerationRadPerSecondSquared);
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        translationVal = xLimiter.calculate(translationVal) * DriveConstants.kMaxSpeedMetersPerSecond;
        strafeVal = yLimiter.calculate(strafeVal) * DriveConstants.kMaxSpeedMetersPerSecond;
        rotationVal =
                rotationLimiter.calculate(rotationVal) * DriveConstants.kMaxAngularSpeedRadiansPerSecond;
                
        /* Drive */
        mDrive.drive(
            new Translation2d(translationVal, strafeVal), 
            rotationVal, 
            true, 
            true
        );
    }
}