package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.constants.robotConfigs.LEFT_FLOWER_GATE;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.LEFT_GATE;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.LEFT_TURRET;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.PANEL;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.RIGHT_FLOWER_GATE;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.RIGHT_GATE;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.RIGHT_TURRET;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(
   name = "Shovel Position Test",
   group = "Test"
)
public class ShovelPositionTest extends LinearOpMode {
   private double lPos = 0.5, rPos = 0.5;
   private Servo L_FLOWER_GATE, R_FLOWER_GATE;

   public void runOpMode() throws InterruptedException {
      L_FLOWER_GATE = hardwareMap.get(Servo.class, LEFT_FLOWER_GATE);
      R_FLOWER_GATE = hardwareMap.get(Servo.class, RIGHT_FLOWER_GATE);

      this.waitForStart();
      L_FLOWER_GATE.setPosition(lPos);
      R_FLOWER_GATE.setPosition(rPos);

      while(this.opModeIsActive()) {
         if (gamepad1.dpadUpWasPressed()){
            lPos += 0.01;
            rPos -= 0.01;
         }
         if (gamepad1.dpadDownWasPressed()){
            lPos -= 0.01;
            rPos += 0.01;
         }

         lPos = Range.clip(lPos, 0, 1);
         rPos = Range.clip(rPos, 0, 1);

         L_FLOWER_GATE.setPosition(lPos);
         R_FLOWER_GATE.setPosition(rPos);

         this.telemetry.addData("L_FLOWER_GATE", lPos);
         this.telemetry.addData("R_FLOWER_GATE", rPos);
         this.telemetry.update();
      }

   }
}
