#include "sensors/bno080sensor.h"

#include "GlobalVars.h"
#include "utils.h"

void BNO080Sensor::motionSetup() {
#ifdef DEBUG_SENSOR
    imu.enableDebugging(Serial);
#endif
    if (!imu.begin(addr, Wire, m_IntPin)) {
        m_Logger.fatal("Can't connect to %s at address 0x%02x",
                       getIMUNameByType(sensorType), addr);
        ledManager.pattern(50, 50, 200);
        return;
    }

    m_Logger.info("Connected to %s on 0x%02x. SW Version: %02x.%02x",
                  getIMUNameByType(sensorType), addr, imu.swMajor, imu.swMinor);

    this->imu.enableLinearAccelerometer(10);
    SlimeVR::Configuration::SensorConfig sensorConfig = configuration.getSensor(sensorId);
    switch (sensorConfig.type) {
        case SlimeVR::Configuration::SensorConfigType::BNO0XX:
            m_Config = sensorConfig.data.bno0XX;
            magStatus = m_Config.magEnabled ? MagnetometerStatus::MAG_ENABLED
                                             : MagnetometerStatus::MAG_DISABLED;
            break;
        default:
            magStatus = USE_6_AXIS ? MagnetometerStatus::MAG_DISABLED
                                   : MagnetometerStatus::MAG_ENABLED;
            break;
    }

    if (!isMagEnabled()) {
        imu.enableGameRotationVector(10);
    } else {
        imu.enableRotationVector(10);
        imu.enableMagnetometer(10);
        imu.enableMagRejection(true);  // Enable magnetic anomaly rejection
    }

    imu.saveCalibrationPeriodically(true);
    imu.requestCalibrationStatus();

    globalTimer.in(60000, [](void* sensor) {
        ((BNO080*)sensor)->sendCalibrateCommand(SH2_CAL_MAG | SH2_CAL_ON_TABLE);
        return true;
    }, &imu);
    
    imu.enableStabilityClassifier(500);
    lastReset = 0;
    lastData = millis();
    working = true;
    configured = true;
    m_tpsCounter.reset();
    m_dataCounter.reset();
}

void BNO080Sensor::motionLoop() {
    m_tpsCounter.update();
    while (imu.dataAvailable()) {
        hadData = true;
        lastData = millis();

        if (isMagEnabled() && imu.getMagAccuracy() < 2) {
            m_Logger.warn("Ignoring low-accuracy magnetometer data");
            continue;  // Skip updates when magnetometer accuracy is low
        }

        if (!isMagEnabled()) {
            if (imu.hasNewGameQuat()) {
                Quat nRotation;
                imu.getGameQuat(nRotation.x, nRotation.y, nRotation.z, nRotation.w, calibrationAccuracy);
                setFusedRotation(nRotation);
            }
        } else {
            if (imu.hasNewQuat()) {
                Quat nRotation;
                imu.getQuat(nRotation.x, nRotation.y, nRotation.z, nRotation.w, magneticAccuracyEstimate, calibrationAccuracy);
                setFusedRotation(nRotation);
            }
        }
    }

    if (lastData + 1000 < millis() && configured) {
        while (true) {
            BNO080Error error = imu.readError();
            if (error.error_source == 255) break;
            lastError = error;
            m_Logger.error("BNO08X error: %d, seq: %d", error.error, error.error_sequence_number);
        }
        working = false;
        lastData = millis();
    }
}

void BNO080Sensor::startCalibration(int calibrationType) {
    imu.sendCalibrateCommand(SH2_CAL_MAG | SH2_CAL_ON_TABLE);
}
