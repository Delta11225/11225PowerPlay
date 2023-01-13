package org.firstinspires.ftc.teamcode.util.nulldevices;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class NullColorSensor implements NormalizedColorSensor {
   @Override
   public NormalizedRGBA getNormalizedColors() {
      return null;
   }

   @Override
   public float getGain() {
      return 0;
   }

   @Override
   public void setGain(float newGain) {

   }

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
}
