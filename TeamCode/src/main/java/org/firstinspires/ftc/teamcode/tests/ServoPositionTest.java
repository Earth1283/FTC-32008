package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.constants.robotConfigs.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/** Check mechanical travel before driving either turret servo independently. */
@TeleOp(
   name = "Servo Position Test",
   group = "Test"
)
public class ServoPositionTest extends LinearOpMode {
   private double x;
   private double y;
   private double rx;
   private double p;
   private String[] names = new String[]{LEFT_GATE, RIGHT_GATE, LEFT_TURRET, RIGHT_TURRET, PANEL, LEFT_FLOWER_GATE, RIGHT_FLOWER_GATE};
   private int index = 0;
   private int servoNum;
   private Servo[] servos;
   private double[] poses;
   private boolean upHold;
   private boolean downHold;

   public ServoPositionTest() {
      this.servoNum = this.names.length;
      this.servos = new Servo[this.servoNum];
      this.poses = new double[this.servoNum];
      this.upHold = false;
      this.downHold = false;
   }

   public void runOpMode() throws InterruptedException {
      for(int i = 0; i < this.servoNum; ++i) {
         this.servos[i] = (Servo)this.hardwareMap.get(Servo.class, this.names[i]);
         this.poses[i] = 0.5;
      }

      this.waitForStart();

      while(this.opModeIsActive()) {
         if (this.gamepad1.dpad_up && !this.upHold && this.index < this.servoNum - 1) {
            this.upHold = true;
            ++this.index;
         } else if (!this.gamepad1.dpad_up) {
            this.upHold = false;
         }

         if (this.gamepad1.dpad_down && !this.downHold && this.index > 0) {
            this.downHold = true;
            --this.index;
         } else if (!this.gamepad1.dpad_down) {
            this.downHold = false;
         }

         if (this.gamepad1.aWasPressed()) {
            double[] var10000 = this.poses;
            int var10001 = this.index;
            var10000[var10001] += 0.01;
         }

         if (this.gamepad1.bWasPressed()) {
            double[] var3 = this.poses;
            int var4 = this.index;
            var3[var4] -= 0.01;
         }

         if (this.gamepad1.right_bumper) {
            applyPosition(this.poses[this.index]);
         }

         if (this.gamepad1.left_bumper) {
            applyPosition(0.0);
         }

         this.telemetry.addData("Current Servo", String.valueOf(this.names[this.index]));

         for(int i = 0; i < this.servoNum; ++i) {
            this.telemetry.addData(i + " " + this.names[i], String.valueOf(this.poses[i]));
         }

         this.telemetry.update();
      }

   }

   private void applyPosition(double position) {
      if (PANEL.equals(names[index])) {
         position = Shooter.limitPanelPosition(position);
         if (!Double.isFinite(position)) return;
         poses[index] = position;
      }
      servos[index].setPosition(position);
   }
}
