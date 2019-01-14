package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

@I2cDeviceType
@DeviceProperties(name="Maxbotix MB1231",description = "Ultrasonic Sensor from Maxbotix",xmlTag = "MB1232")
public class MaxBotixUltrasonic extends I2cDeviceSynchDevice<I2cDeviceSynch> {
    public final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create8bit(0xE0);
    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    protected boolean doInitialize() {
        return true;
    }

    @Override
    public String getDeviceName() {
        return "MaxBotix Ultrasonic Sensor";
    }

    public MaxBotixUltrasonic(I2cDeviceSynch deviceClient) {
        super(deviceClient,true);

        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);
        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    public enum register {
        
    }

}
