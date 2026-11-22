package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannel.Mode;
import com.qualcomm.robotcore.hardware.Servo;

/** Requires a test configuration with motors m0–m7 and servos s0–s6. */
@TeleOp
public class DeviceTest extends LinearOpMode {
   private int motorNum = 8;
   private int servoNum = 7;
   private int touchNum = 0;
   private DcMotorEx[] motors;
   private Servo[] servos;
   private DigitalChannel[] touches;
   private int servoIndex;
   private int motorIndex;
   private boolean upHold;
   private boolean downHold;
   private boolean yHold;
   private boolean aHold;

   public DeviceTest() {
      this.motors = new DcMotorEx[this.motorNum];
      this.servos = new Servo[this.servoNum];
      this.touches = new DigitalChannel[this.touchNum];
      this.servoIndex = 0;
      this.motorIndex = 0;
      this.upHold = false;
      this.downHold = false;
      this.yHold = false;
      this.aHold = false;
   }

   public void runOpMode() {
      for(int i = 0; i < this.motorNum; ++i) {
         this.motors[i] = (DcMotorEx)this.hardwareMap.get(DcMotorEx.class, "m" + i);
      }

      for(int i = 0; i < this.servoNum; ++i) {
         this.servos[i] = (Servo)this.hardwareMap.get(Servo.class, "s" + i);
      }

      for(int i = 0; i < this.touchNum; ++i) {
         this.touches[i] = (DigitalChannel)this.hardwareMap.get(DigitalChannel.class, "t" + i);
         this.touches[i].setMode(Mode.INPUT);
      }

      this.waitForStart();

      while(this.opModeIsActive()) {
         if (this.gamepad1.dpad_up && !this.upHold) {
            this.upHold = true;
            ++this.servoIndex;
         } else if (!this.gamepad1.dpad_up) {
            this.upHold = false;
         }

         if (this.gamepad1.dpad_down && !this.downHold) {
            this.downHold = true;
            --this.servoIndex;
         } else if (!this.gamepad1.dpad_down) {
            this.downHold = false;
         }

         if (this.gamepad1.y && !this.yHold) {
            this.yHold = true;
            ++this.motorIndex;
         } else if (!this.gamepad1.y) {
            this.yHold = false;
         }

         if (this.gamepad1.a && !this.aHold) {
            this.aHold = true;
            --this.motorIndex;
         } else if (!this.gamepad1.a) {
            this.aHold = false;
         }

         if (this.gamepad1.right_bumper) {
            if (this.motorIndex >= this.motorNum) {
               this.motorIndex = this.motorNum - 1;
            } else if (this.motorIndex < 0) {
               this.motorIndex = 0;
            }

            this.motors[this.motorIndex].setPower(1.0);
         } else {
            for(int i = 0; i < this.motorNum; ++i) {
               this.motors[i].setPower(0.0);
            }
         }

         if (this.gamepad1.left_bumper) {
            if (this.servoIndex >= this.servoNum) {
               this.servoIndex = this.servoNum - 1;
            } else if (this.servoIndex < 0) {
               this.servoIndex = 0;
            }

            this.servos[this.servoIndex].setPosition(1.0);
         } else if (this.gamepad1.left_trigger > 0.0F) {
            if (this.servoIndex >= this.servoNum) {
               this.servoIndex = this.servoNum - 1;
            } else if (this.servoIndex < 0) {
               this.servoIndex = 0;
            }

            this.servos[this.servoIndex].setPosition(0.0);
         }

         for(int i = 0; i < this.touchNum && this.opModeIsActive(); ++i) {
            if (this.touches[i].getState()) {
               this.telemetry.addData("Digital Touch" + i, "Is Not Pressed");
            } else {
               this.telemetry.addData("Digital Touch" + i, "Is Pressed");
            }
         }

         this.telemetry.addData("Servo", "s" + this.servoIndex);
         this.telemetry.addData("Motor", "m" + this.motorIndex);
         this.telemetry.update();
      }

   }
}
