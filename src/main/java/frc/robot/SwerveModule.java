package frc.robot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants.Swerve;

public class SwerveModule extends SubsystemBase{
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANcoder angleEncoder;
    
    private final FlywheelSim mAzimuthSim =
        new FlywheelSim(LinearSystemId.identifyVelocitySystem(0.007, 0.00008), DCMotor.getFalcon500(1), Swerve.chosenModule.angleGearRatio); 

    private final FlywheelSim mDriveWheelSim =
            new FlywheelSim( LinearSystemId.identifyVelocitySystem(2.75, 0.26), DCMotor.getFalcon500(1), Swerve.chosenModule.driveGearRatio);

    
    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    /* drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);

    /* angle motor control requests */
    private final PositionVoltage anglePosition = new PositionVoltage(0);
    private double mDriveWheelSimDistance, mAzimuthSimDistance = 0;
    private BaseStatusSignal[] signals;

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID, moduleConstants.CANbusName);
        configAngleEncoder();
        

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID, moduleConstants.CANbusName);
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID, moduleConstants.CANbusName);
        configDriveMotor();


        signals = new BaseStatusSignal[4];

        signals[0] = mDriveMotor.getPosition();
        signals[1] = mDriveMotor.getVelocity();
        signals[2] = mAngleMotor.getPosition();
        signals[3] = mAngleMotor.getVelocity();

        lastAngle = getState().angle;



    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); 
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.setControl(driveDutyCycle);
        }
        else {
            //driveVelocity.EnableFOC = false;
            //driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
            mDriveMotor.setControl(new VelocityVoltage(Conversions.MPSToTalon(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio)).withFeedForward(driveFeedForward.calculate(desiredState.speedMetersPerSecond)));
        }
    }
    

    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? getAngle() : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        
        anglePosition.Position = Conversions.degreesToTalon(angle.getDegrees(), Constants.Swerve.angleGearRatio);
        mAngleMotor.setControl(anglePosition);
        lastAngle = angle;
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(Conversions.talonToDegrees(mAngleMotor.getPosition().getValue(), Constants.Swerve.angleGearRatio));
    }

    public Rotation2d getCANcoder(){
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
    }

    private Rotation2d waitForCANcoder(){
        /* wait for up to 250ms for a new CANcoder position */
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().waitForUpdate(0.250).getValue());
    }

    public void resetToAbsolute(){
        double absolutePosition = Conversions.degreesToTalon(waitForCANcoder().getDegrees() - angleOffset.getDegrees(), Constants.Swerve.angleGearRatio);
        mAngleMotor.setRotorPosition(absolutePosition);
    }

    private void configAngleEncoder(){    
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);
    }

    private void configAngleMotor(){
        mAngleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);
        resetToAbsolute();
    }

    private void configDriveMotor(){
        mDriveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.getConfigurator().setRotorPosition(0);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.talonToMPS(mDriveMotor.getVelocity().getValue(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio), 
            getAngle()
        ); 
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.talonToMeters(mDriveMotor.getPosition().getValue(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio), 
            getAngle()
        );
    }


    @Override
    public void simulationPeriodic(){
        
        mDriveMotor.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());
        mAngleMotor.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());
        angleEncoder.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());

        mAzimuthSim.setInputVoltage(mAngleMotor.getDutyCycle().getValue() * RobotController.getBatteryVoltage());
        mDriveWheelSim.setInputVoltage(mDriveMotor.getDutyCycle().getValue() * RobotController.getBatteryVoltage());

        mAzimuthSim.update(0.02);
        mDriveWheelSim.update(0.02);

        final double kTurnDistancePerPulse = 360.0 / (12.8);
        final double kDriveDistancePerPulse = (Units.inchesToMeters(3.95) * Math.PI) / (6.75);

        double angleVel = mAzimuthSim.getAngularVelocityRadPerSec() / kTurnDistancePerPulse;
        double driveVel = mDriveWheelSim.getAngularVelocityRadPerSec() / kDriveDistancePerPulse;

        mAngleMotor.getSimState().setRotorVelocity(angleVel);
        mDriveMotor.getSimState().setRotorVelocity(driveVel);
        
        mAzimuthSimDistance += mAzimuthSim.getAngularVelocityRadPerSec() * 0.02;
        mDriveWheelSimDistance += mDriveWheelSim.getAngularVelocityRadPerSec() * 0.02;

        double anglePos = mAzimuthSimDistance / kTurnDistancePerPulse; //Conversions.degreesToTalon(mAzimuthSimDistance, Constants.Swerve.angleGearRatio);
        double drivePos = mDriveWheelSimDistance / kDriveDistancePerPulse; //Conversions.MetersToTalon(mDriveWheelSimDistance, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);

        mAngleMotor.getSimState().setRawRotorPosition(anglePos);
        mDriveMotor.getSimState().setRawRotorPosition(drivePos);

        }

        public BaseStatusSignal[] getSignals(){
            return signals;
        }

}