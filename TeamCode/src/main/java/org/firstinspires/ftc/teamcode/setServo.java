package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class setServo extends LinearOpMode {

    Servo servo;

    @Override
    public void runOpMode(){
        servo = hardwareMap.get(Servo.class, "servoCapstone");

        waitForStart();
        servo.setPosition(0.5);
        sleep(10000);
    }
}
