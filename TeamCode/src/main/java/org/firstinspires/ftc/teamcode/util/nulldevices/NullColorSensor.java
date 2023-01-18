package org.firstinspires.ftc.teamcode.util.nulldevices;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class NullColorSensor implements ColorSensor {
   @Override
   public Manufacturer getManufacturer() {
      return null;
   }

   @Override
   public String getDeviceName() {
      return null;
   }

   @Override
   public String getConnectionInfo() {
      return null;
   }

   @Override
   public int getVersion() {
      return 0;
   }

   @Override
   public void resetDeviceConfigurationForOpMode() {

   }

   @Override
   public void close() {

   }

   @Override
   public int red() {
      return 0;
   }

   @Override
   public int green() {
      return -1;
   }

   @Override
   public int blue() {
      return -1;
   }

   @Override
   public int alpha() {
      return -1;
   }

   @Override
   public int argb() {
      return -1;
   }

   @Override
   public void enableLed(boolean enable) {

   }

   @Override
   public void setI2cAddress(I2cAddr newAddress) {

   }

   @Override
   public I2cAddr getI2cAddress() {
      return null;
   }
}
