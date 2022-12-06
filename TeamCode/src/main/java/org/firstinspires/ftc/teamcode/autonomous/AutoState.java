package org.firstinspires.ftc.teamcode.autonomous;

public class AutoState {
   public final Color color;
   public final StartPosition position;
   public final long delay;

   AutoState(Color color, StartPosition position, long delay) {
      this.color = color;
      this.position = position;
      this.delay = delay;
   }
}
