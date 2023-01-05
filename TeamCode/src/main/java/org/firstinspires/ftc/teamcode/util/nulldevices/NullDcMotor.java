package org.firstinspires.ftc.teamcode.util.nulldevices;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class NullDcMotor implements DcMotor {
   @Override
   public MotorConfigurationType getMotorType() {
      return null;
   }

   @Override
   public void setMotorType(MotorConfigurationType motorConfigurationType) {

   }

   @Override
   public DcMotorController getController() {
      return null;
   }

   @Override
   public int getPortNumber() {
      return 0;
   }

   @Override
   public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {

   }

   @Override
   public ZeroPowerBehavior getZeroPowerBehavior() {
      return null;
   }

   @Override
   public void setPowerFloat() {

   }

   @Override
   public boolean getPowerFloat() {
      return false;
   }

   @Override
   public void setTargetPosition(int i) {

   }

   @Override
   public int getTargetPosition() {
      return 0;
   }

   @Override
   public boolean isBusy() {
      return false;
   }

   @Override
   public int getCurrentPosition() {
      return 0;
   }

   @Override
   public void setMode(RunMode runMode) {

   }

   @Override
   public RunMode getMode() {
      return null;
   }

   @Override
   public void setDirection(Direction direction) {

   }

   @Override
   public Direction getDirection() {
      return null;
   }

   @Override
   public void setPower(double v) {

   }

   @Override
   public double getPower() {
      return 0;
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
