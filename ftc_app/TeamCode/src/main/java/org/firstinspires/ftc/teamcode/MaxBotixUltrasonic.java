/*
Copyright 2019 FIRST Tech Challenge Team 11792
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

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
