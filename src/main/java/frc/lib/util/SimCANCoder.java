package frc.lib.util;

import java.util.ArrayList;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.jni.PlatformJNI;
import com.ctre.phoenix6.sim.DeviceType;
import com.ctre.phoenix6.wpiutils.CallbackHelper;

import edu.wpi.first.hal.HALValue;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.hal.simulation.SimValueCallback;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.HAL.SimPeriodicBeforeCallback;
import edu.wpi.first.wpilibj.simulation.CallbackStore;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

public class SimCANCoder extends CANcoder {

	private SimDevice m_simCANCoder;
	private SimDouble m_simVbat;
	private SimDouble m_simPosition;
	private SimDouble m_simAbsPosition;
	private SimDouble m_simRawPosition;
	private SimDouble m_simVelocity;
    private static StatusSignal<Double> posSignal, velSignal;

	// callbacks to register
	private SimValueCallback onValueChangedCallback = new OnValueChangedCallback();
	private Runnable onPeriodicCallback = new OnPeriodicCallback();

	// returned registered callbacks
	private ArrayList<CallbackStore> simValueChangedCallbacks = new ArrayList<CallbackStore>();
	private SimPeriodicBeforeCallback simPeriodicCallback;

	/**
	 * Constructor for CANCoder
	 * @param deviceNumber device ID of CANCoder
	 * @param canbus Name of the CANbus; can be a CANivore device name or serial number.
	 *               Pass in nothing or "rio" to use the roboRIO.
	 */
	public SimCANCoder(int deviceNumber, String canbus) {
		super(deviceNumber, canbus);
		SendableRegistry.addLW(this, "CANCoder ", deviceNumber);

		m_simCANCoder = SimDevice.create("CANEncoder:CANCoder", deviceNumber);
		if(m_simCANCoder != null){
			simPeriodicCallback = HAL.registerSimPeriodicBeforeCallback(onPeriodicCallback);

			m_simVbat = m_simCANCoder.createDouble("busVoltage", Direction.kInput, 12.0);

			m_simPosition = m_simCANCoder.createDouble("position", Direction.kOutput, 0);
			m_simAbsPosition = m_simCANCoder.createDouble("absolutePosition", Direction.kOutput, 0);

			m_simRawPosition = m_simCANCoder.createDouble("rawPositionInput", Direction.kInput, 0);
			m_simVelocity = m_simCANCoder.createDouble("velocity", Direction.kInput, 0);

			SimDeviceSim sim = new SimDeviceSim("CANEncoder:CANCoder");
			simValueChangedCallbacks.add(sim.registerValueChangedCallback(m_simVbat, onValueChangedCallback, true));
			simValueChangedCallbacks.add(sim.registerValueChangedCallback(m_simRawPosition, onValueChangedCallback, true));
			simValueChangedCallbacks.add(sim.registerValueChangedCallback(m_simVelocity, onValueChangedCallback, true));
		}

        posSignal = getPosition();
        velSignal = getVelocity();
	}

	/**
	 * Constructor for CANCoder
	 * @param deviceNumber device ID of CANCoder
	 */
	public SimCANCoder(int deviceNumber) {
		this(deviceNumber, "");
	}

	// ----- Auto-Closable ----- //
	@Override
	public void close(){
		SendableRegistry.remove(this);
		if(m_simCANCoder != null) {
			m_simCANCoder.close();
			m_simCANCoder = null;
		}
		super.close(); //Destroy the device
	}

	// ----- Callbacks for Sim ----- //
	private class OnValueChangedCallback implements SimValueCallback {
		@Override
		public void callback(String name, int handle, int direction, HALValue value) {
			String deviceName = SimDeviceDataJNI.getSimDeviceName(SimDeviceDataJNI.getSimValueDeviceHandle(handle));
			String physType = deviceName + ":" + name;
			PlatformJNI.JNI_SimSetPhysicsInput(DeviceType.CANCoderType.value, getDeviceID(),
											   physType, CallbackHelper.getRawValue(value));
		}
	}

	private class OnPeriodicCallback implements Runnable {
		@Override
		public void run() {
			double value = 0;
			int err = 0;
			int deviceID = getDeviceID();

			value = PlatformJNI.JNI_SimGetPhysicsValue(DeviceType.CANCoderType.value, deviceID, "BusVoltage");
			err = PlatformJNI.JNI_SimGetLastError(DeviceType.CANCoderType.value, deviceID);
			if (err == 0) {
				m_simVbat.set(value);
			}
			value = PlatformJNI.JNI_SimGetPhysicsValue(DeviceType.CANCoderType.value, deviceID, "IntegSensPos");
			err = PlatformJNI.JNI_SimGetLastError(DeviceType.CANCoderType.value, deviceID);
			if (err == 0) {
				m_simPosition.set(value);
			}
			value = PlatformJNI.JNI_SimGetPhysicsValue(DeviceType.CANCoderType.value, deviceID, "IntegSensAbsPos");
			err = PlatformJNI.JNI_SimGetLastError(DeviceType.CANCoderType.value, deviceID);
			if (err == 0) {
				m_simAbsPosition.set(value);
			}
			value = PlatformJNI.JNI_SimGetPhysicsValue(DeviceType.CANCoderType.value, deviceID, "IntegSensRawPos");
			err = PlatformJNI.JNI_SimGetLastError(DeviceType.CANCoderType.value, deviceID);
			if (err == 0) {
				m_simRawPosition.set(value);
			}
			value = PlatformJNI.JNI_SimGetPhysicsValue(DeviceType.CANCoderType.value, deviceID, "IntegSensVel");
			err = PlatformJNI.JNI_SimGetLastError(DeviceType.CANCoderType.value, deviceID);
			if (err == 0) {
				m_simVelocity.set(value);
			}

            
    
		}
	}

    private double getSensorPosition(){
        
        posSignal.refresh();
        velSignal.refresh();
        double val = BaseStatusSignal.getLatencyCompensatedValue(posSignal, velSignal);
        return val;

    }

	// ----- Sendable ----- //
	@Override
  	public void initSendable(SendableBuilder builder) {
		builder.setSmartDashboardType("CANCoder");
		builder.addDoubleProperty("Position", this::getSensorPosition, null);
  	}
}
