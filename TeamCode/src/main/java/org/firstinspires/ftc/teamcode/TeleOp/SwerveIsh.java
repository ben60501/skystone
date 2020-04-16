package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "SwerveIsh", group = "TeleOp tests")
@Disabled
public class SwerveIsh extends OpMode {

    private DcMotor motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight;
    private double leftX, rightX, leftY, rightY, turnValue;

    @Override
    public void init() {

        motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorFrontLeft = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");
        motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");

        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
    }

    @Override
    public void loop() {

        leftX = gamepad1.left_stick_x;
        rightX = gamepad1.right_stick_x;
        leftY = -gamepad1.left_stick_y;
        rightY = -gamepad1.right_stick_y;

         if (Math.abs(rightY) >= 0.1 || Math.abs(rightX) >= 0.1) {
            if (Math.abs(rightY) >= Math.abs(rightX)) {
                //Move Y
                if(rightY > 0 && leftX > 0.5){
                    motorFrontRight.setPower(rightY/4);
                    motorFrontLeft.setPower(rightY);
                    motorBackRight.setPower(rightY/4);
                    motorBackLeft.setPower(rightY);
                }else if(rightY > 0 && leftX < -0.5){
                    motorFrontRight.setPower(rightY);
                    motorFrontLeft.setPower(-rightY/2);
                    motorBackRight.setPower(rightY);
                    motorBackLeft.setPower(-rightY/2);
                }else if(rightY < 0 && leftX > 0.5) {
                    motorFrontRight.setPower(rightY);
                    motorFrontLeft.setPower(-rightY/2);
                    motorBackRight.setPower(rightY);
                    motorBackLeft.setPower(-rightY/2);
                }else if(rightY < 0 && leftX < -0.5) {
                    motorFrontRight.setPower(-rightY/2);
                    motorFrontLeft.setPower(rightY);
                    motorBackRight.setPower(-rightY/2);
                    motorBackLeft.setPower(rightY);
                }else {
                    motorFrontRight.setPower(rightY);
                    motorFrontLeft.setPower(rightY);
                    motorBackRight.setPower(rightY);
                    motorBackLeft.setPower(rightY);
                }
            } else {
                //Move X
                motorFrontRight.setPower(-rightX);
                motorFrontLeft.setPower(rightX);
                motorBackRight.setPower(rightX);
                motorBackLeft.setPower(-rightX);
            }
        } else {
            motorFrontRight.setPower(0);
            motorFrontLeft.setPower(0);
            motorBackRight.setPower(0);
            motorBackLeft.setPower(0);
        }

    }

}
