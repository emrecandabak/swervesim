package frc.lib.util;

import java.util.ArrayList;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.jni.PlatformJNI;
import com.ctre.phoenix6.sim.DeviceType;
import com.ctre.phoenix6.wpiutils.AutoFeedEnable;
import com.ctre.phoenix6.wpiutils.CallbackHelper;

import edu.wpi.first.wpilibj.simulation.CallbackStore;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.hal.simulation.SimValueCallback;
import edu.wpi.first.hal.HAL.SimPeriodicBeforeCallback;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.HALValue;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;

/**
 * CTRE Talon FX Motor Controller when used on CAN Bus.
 */
public class SimTalonFX extends TalonFX {

	private String _description;
	private SimDevice m_simMotor;
	private SimDouble m_simPercOut;
	private SimDouble m_simMotorOutputLeadVoltage;
	private SimDouble m_simSupplyCurrent;
	private SimDouble m_simMotorCurrent;
	private SimDouble m_simVbat;

	private SimDevice m_simIntegSens;
	private SimDouble m_simIntegSensPos;
	private SimDouble m_simIntegSensAbsPos;
	private SimDouble m_simIntegSensRawPos;
	private SimDouble m_simIntegSensVel;


	// callbacks to register
	private SimValueCallback onValueChangedCallback = new OnValueChangedCallback();
	private Runnable onPeriodicCallback = new OnPeriodicCallback();

	// returned registered callbacks
    private SimPeriodicBeforeCallback simPeriodicCallback;
	private ArrayList<CallbackStore> simValueChangedCallbacks = new ArrayList<CallbackStore>();
	/**
	 * Constructor for motor controller
	 * @param deviceNumber device ID of motor controller
	 * @param canbus Name of the CANbus; can be a CANivore device name or serial number.
	 *               Pass in nothing or "rio" to use the roboRIO.
	 */
	public SimTalonFX(int deviceNumber, String canbus) {
		super(deviceNumber, canbus);
		_description = "Talon FX " + deviceNumber;

		SendableRegistry.addLW(this, "Talon FX ", deviceNumber);

		m_simMotor = SimDevice.create("CANMotor:Talon FX", deviceNumber);
		if(m_simMotor != null){
			AutoFeedEnable.getInstance();
			simPeriodicCallback = HAL.registerSimPeriodicBeforeCallback(onPeriodicCallback);

			m_simPercOut = m_simMotor.createDouble("percentOutput", Direction.kOutput, 0);
			m_simMotorOutputLeadVoltage = m_simMotor.createDouble("motorOutputLeadVoltage", Direction.kOutput, 0);

			m_simSupplyCurrent = m_simMotor.createDouble("supplyCurrent", Direction.kInput, 0);
			m_simMotorCurrent = m_simMotor.createDouble("motorCurrent", Direction.kInput, 0);
			m_simVbat = m_simMotor.createDouble("busVoltage", Direction.kInput, 12.0);

			SimDeviceSim sim = new SimDeviceSim("CANMotor:Talon FX");
			simValueChangedCallbacks.add(sim.registerValueChangedCallback(m_simSupplyCurrent, onValueChangedCallback, true));
			simValueChangedCallbacks.add(sim.registerValueChangedCallback(m_simMotorCurrent, onValueChangedCallback, true));
			simValueChangedCallbacks.add(sim.registerValueChangedCallback(m_simVbat, onValueChangedCallback, true));
		}

		String base = "Talon FX[" + deviceNumber + "]/";
		m_simIntegSens = SimDevice.create("CANEncoder:" + base + "Integrated Sensor");
		if(m_simIntegSens != null){
			m_simIntegSensPos = m_simIntegSens.createDouble("position", Direction.kOutput, 0);
			m_simIntegSensAbsPos = m_simIntegSens.createDouble("absolutePosition", Direction.kOutput, 0);

			m_simIntegSensRawPos = m_simIntegSens.createDouble("rawPositionInput", Direction.kInput, 0);
			m_simIntegSensVel = m_simIntegSens.createDouble("velocity", Direction.kInput, 0);

			SimDeviceSim sim = new SimDeviceSim("CANEncoder:" + base + "Integrated Sensor");
			simValueChangedCallbacks.add(sim.registerValueChangedCallback(m_simIntegSensRawPos, onValueChangedCallback, true));
			simValueChangedCallbacks.add(sim.registerValueChangedCallback(m_simIntegSensVel, onValueChangedCallback, true));
		}

        System.out.println("running!");
        super.getSimState();
	}
	

	/**
	 * Constructor for motor controller
	 * @param deviceNumber device ID of motor controller
	 */
	public SimTalonFX(int deviceNumber) {
		this(deviceNumber, "");
	}

    // ----- Auto-Closable ----- //
    @Override
    public void close(){
        SendableRegistry.remove(this);
        if(m_simMotor != null) {
            m_simMotor.close();
            m_simMotor = null;
        }
        if(m_simIntegSens != null) {
            m_simIntegSens.close();
            m_simIntegSens = null;
        }
        super.close(); //Destroy the device
    }

	// ----- Callbacks for Sim ----- //
	private class OnValueChangedCallback implements SimValueCallback {
		@Override
		public void callback(String name, int handle, int direction, HALValue value) {
			String deviceName = SimDeviceDataJNI.getSimDeviceName(SimDeviceDataJNI.getSimValueDeviceHandle(handle));
			String physType = deviceName + ":" + name;
			PlatformJNI.JNI_SimSetPhysicsInput(DeviceType.TalonFXType.value, getDeviceID(),
								               physType, CallbackHelper.getRawValue(value));
		}
	}

	private class OnPeriodicCallback implements Runnable {
		@Override
		public void run() {
			double value = 0;
			int err = 0;

			int deviceID = getDeviceID();

			value = PlatformJNI.JNI_SimGetPhysicsValue(DeviceType.TalonFXType.value, deviceID, "PercentOutput");
			err = PlatformJNI.JNI_SimGetLastError(DeviceType.TalonFXType.value, deviceID);
			if (err == 0) {
				m_simPercOut.set(value);
			}
			value = PlatformJNI.JNI_SimGetPhysicsValue(DeviceType.TalonFXType.value, deviceID, "MotorOutputLeadVoltage");
			err = PlatformJNI.JNI_SimGetLastError(DeviceType.TalonFXType.value, deviceID);
			if (err == 0) {
				m_simMotorOutputLeadVoltage.set(value);
			}
			value = PlatformJNI.JNI_SimGetPhysicsValue(DeviceType.TalonFXType.value, deviceID, "BusVoltage");
			err = PlatformJNI.JNI_SimGetLastError(DeviceType.TalonFXType.value, deviceID);
			if (err == 0) {
				m_simVbat.set(value);
			}
			value = PlatformJNI.JNI_SimGetPhysicsValue(DeviceType.TalonFXType.value, deviceID, "CurrentSupply");
			err = PlatformJNI.JNI_SimGetLastError(DeviceType.TalonFXType.value, deviceID);
			if (err == 0) {
				m_simSupplyCurrent.set(value);
			}
			value = PlatformJNI.JNI_SimGetPhysicsValue(DeviceType.TalonFXType.value, deviceID, "CurrentStator");
			err = PlatformJNI.JNI_SimGetLastError(DeviceType.TalonFXType.value, deviceID);
			if (err == 0) {
				m_simMotorCurrent.set(value);
			}
			value = PlatformJNI.JNI_SimGetPhysicsValue(DeviceType.TalonFXType.value, deviceID, "IntegSensPos");
			err = PlatformJNI.JNI_SimGetLastError(DeviceType.TalonFXType.value, deviceID);
			if (err == 0) {
				m_simIntegSensPos.set(value);
			}
			value = PlatformJNI.JNI_SimGetPhysicsValue(DeviceType.TalonFXType.value, deviceID, "IntegSensAbsPos");
			err = PlatformJNI.JNI_SimGetLastError(DeviceType.TalonFXType.value, deviceID);
			if (err == 0) {
				m_simIntegSensAbsPos.set(value);
			}
			value = PlatformJNI.JNI_SimGetPhysicsValue(DeviceType.TalonFXType.value, deviceID, "IntegSensRawPos");
			err = PlatformJNI.JNI_SimGetLastError(DeviceType.TalonFXType.value, deviceID);
			if (err == 0) {
				m_simIntegSensRawPos.set(value);
			}
			value = PlatformJNI.JNI_SimGetPhysicsValue(DeviceType.TalonFXType.value, deviceID, "IntegSensVel");
			err = PlatformJNI.JNI_SimGetLastError(DeviceType.TalonFXType.value, deviceID);
			if (err == 0) {
				m_simIntegSensVel.set(value);
			}

            
            
		}
	}

	/**
	 * Common interface for returning the inversion state of a speed controller.
	 *
	 * @return The state of inversion, true is inverted.
	 */
	@Override
	public boolean getInverted() {
		return super.getInverted();
	}

	// ----------------------- turn-motor-off routines-------------------//
	/**
	 * Common interface for disabling a motor.
	 */
	@Override
	public void disable() {
		super.disable();
	}

	/**
	 * Common interface to stop the motor until Set is called again.
	 */
	@Override
	public void stopMotor() {
		stopMotor();
	}

	// ---- Sendable -------//

	/**
	 * Initialize sendable
	 * @param builder Base sendable to build on
	 */
	@Override
	public void initSendable(SendableBuilder builder) {
		builder.setSmartDashboardType("Motor Controller");
		builder.setActuator(true);
		builder.setSafeState(this::stopMotor);
		builder.addDoubleProperty("Value", this::get, this::set);
	}

	/**
	 * @return description of controller
	 */
	public String getDescription() {
		return _description;
	}

    public void simPeriodic(){

    }

}

