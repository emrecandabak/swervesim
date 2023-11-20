package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.stream.DoubleStream;
import java.util.stream.Stream;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.unmanaged.Unmanaged;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    private static double mSimHeading = 0;
    private Field2d mField2d = new Field2d();
    private OdometryUpdateThread odometryUpdateThread;
    private final double kSkewCompensationCoefficient = 0.2215;

    public Drive() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        zeroGyro();

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getHeadingRotation(), getModulePositions());
        odometryUpdateThread = new OdometryUpdateThread();
        odometryUpdateThread.start();
        mField2d.setRobotPose(getPose());
        
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        final double yCompensator = (kSkewCompensationCoefficient * rotation * Math.signum(translation.getX()));
        SwerveModuleState[] swerveModuleStates =    
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY() + yCompensator , 
                                    rotation, 
                                    getPose().getRotation()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }

        SmartDashboard.putNumberArray(
            "DesiredStates",
            new double[] {
                swerveModuleStates[0].angle.getDegrees(),
                swerveModuleStates[0].speedMetersPerSecond,
                swerveModuleStates[1].angle.getDegrees(),
                swerveModuleStates[1].speedMetersPerSecond,
                swerveModuleStates[2].angle.getDegrees(),
                swerveModuleStates[2].speedMetersPerSecond,
                swerveModuleStates[3].angle.getDegrees(),
                swerveModuleStates[3].speedMetersPerSecond
            });

    }    

    /* Used by SwerveControllerCommand in Auto */

    public void setModuleStates(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] desiredStates =
                Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }

        SmartDashboard.putNumberArray(
            "DesiredStates",
            new double[] {
                desiredStates[0].angle.getDegrees(),
                desiredStates[0].speedMetersPerSecond,
                desiredStates[1].angle.getDegrees(),
                desiredStates[1].speedMetersPerSecond,
                desiredStates[2].angle.getDegrees(),
                desiredStates[2].speedMetersPerSecond,
                desiredStates[3].angle.getDegrees(),
                desiredStates[3].speedMetersPerSecond
            });
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getHeadingRotation(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        gyro.setYaw(0);
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw().getValue()) : Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public Rotation2d getHeadingRotation() {
        gyro.getYaw().refresh();
        return Rotation2d.fromDegrees(
                Math.IEEEremainder(gyro.getAngle(), 360)
                        * (true ? -1.0 : 1.0));
    }

    @Override
    public void periodic(){
        //swerveOdometry.update(getHeadingRotation(), getModulePositions());  
        mField2d.setRobotPose(getPose());
        SmartDashboard.putNumber("Gyro", getHeadingRotation().getDegrees());
        SmartDashboard.putData(mField2d);
        SmartDashboard.putNumberArray(
                "MeasuredStates",
                new double[] {
                    mSwerveMods[0].getState().angle.getDegrees(),
                    mSwerveMods[0].getState().speedMetersPerSecond,
                    mSwerveMods[1].getState().angle.getDegrees(),
                    mSwerveMods[1].getState().speedMetersPerSecond,
                    mSwerveMods[2].getState().angle.getDegrees(),
                    mSwerveMods[2].getState().speedMetersPerSecond,
                    mSwerveMods[3].getState().angle.getDegrees(),
                    mSwerveMods[3].getState().speedMetersPerSecond
                });
    }

    
    @Override
    public void simulationPeriodic() {
        Unmanaged.feedEnable(20);
        
        
        mSimHeading += Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates()).omegaRadiansPerSecond
                        * 0.02;

        gyro.getSimState().setRawYaw(Units.radiansToDegrees(mSimHeading));

        
    }

    private class OdometryUpdateThread extends Thread {
        private BaseStatusSignal[] allSignals;
        public int successfulDataAcquisitions = 0;
        public int failedDataAcquisitions = 0;

        private LinearFilter lowpass = LinearFilter.movingAverage(50);
        private double lastTime = 0;
        private double currentTime = 0;
        private double averageLoopTime = 0;

        public OdometryUpdateThread() {
            ArrayList<BaseStatusSignal> signalsList = new ArrayList<>();
            allSignals = new BaseStatusSignal[(4 * 4) + 2];
            for (int i = 0; i < 4; i++) {
                signalsList.addAll(Arrays.asList(mSwerveMods[i].getSignals()));
            }
        
            signalsList.addAll(Arrays.asList(gyro.getYaw()));
            allSignals = signalsList.toArray(new BaseStatusSignal[0]);
        }

        public void run() {
            for (var sig : allSignals) {
                if (sig instanceof StatusSignal) {
                    ((StatusSignal<?>) sig).setUpdateFrequency(250);
                    
                }
                System.out.println(((StatusSignal<?>)sig).toString());
            }
            while (true) {
                
                var status = BaseStatusSignal.waitForAll(0.1, allSignals);
                lastTime = currentTime;
                currentTime = Utils.getCurrentTimeSeconds();
                averageLoopTime = lowpass.calculate(currentTime - lastTime);
                if (status.isOK()) {
                    successfulDataAcquisitions++;
                } else {
                    failedDataAcquisitions++;
                }


                synchronized (swerveOdometry) {
                    synchronized (getModulePositions()) {
                        synchronized (getHeadingRotation()) {
                            swerveOdometry.update(getHeadingRotation(), getModulePositions());
                        }
                    }
                }


            }
        }

        public double getTime() {
            return averageLoopTime;
        }

        public int getSuccessfulDataAcquisitions() {
            return successfulDataAcquisitions;
        }

        public int getFailedDataAcquisitions() {
            return failedDataAcquisitions;
        }

        
    }


}