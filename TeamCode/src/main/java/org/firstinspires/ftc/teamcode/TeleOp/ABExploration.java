package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Disabled
public class ABExploration extends OpMode {

    private DcMotor motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight, motorOuttake, motorIntakeRight, motorIntakeLeft;
    private Servo servoIntake, servoOuttakeRotate, servoOuttakeClaw, servoFoundationLeft, servoFoundationRight;

    private double leftX, rightX, leftY, rightY, leftX2, leftY2;

    @Override
    public void init() {
        motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorFrontLeft = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");
        motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorOuttake = hardwareMap.get(DcMotor.class, "motorOuttake");
        motorIntakeLeft = hardwareMap.get(DcMotor.class, "motorIntakeLeft");
        motorIntakeRight = hardwareMap.get(DcMotor.class, "motorIntakeRight");

        servoIntake = hardwareMap.get(Servo.class, "servoIntake");
        servoOuttakeClaw = hardwareMap.get(Servo.class, "servoOuttakeClaw");
        servoOuttakeRotate = hardwareMap.get(Servo.class, "servoOuttakeRotate");
        servoFoundationLeft = hardwareMap.get(Servo.class, "servoFoundationLeft");
        servoFoundationRight = hardwareMap.get(Servo.class, "servoFoundationRight");

        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorOuttake.setDirection(DcMotor.Direction.REVERSE);
        motorIntakeRight.setDirection(DcMotor.Direction.REVERSE);
        motorIntakeLeft.setDirection(DcMotor.Direction.REVERSE);

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
        motorOuttake.setPower(0);
        motorIntakeRight.setPower(0);
        motorIntakeLeft.setPower(0);

        motorOuttake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorOuttake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorOuttake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    @Override
    public void loop() {
        leftX = gamepad1.left_stick_x;
        rightX = gamepad1.right_stick_x;
        leftY = -gamepad1.left_stick_y;
        rightY = -gamepad1.right_stick_y;

        leftX2 = gamepad2.left_stick_x;
        leftY2 = -gamepad2.left_stick_y;

        if (gamepad1.right_bumper) {
            leftX *= 0.5;
            rightX *= 0.5;
            leftY *= 0.5;
            rightY *= 0.5;
        }

        //Drivetrain
        if (Math.abs(rightX) >= 0.1) {
            //Turning
            motorFrontRight.setPower(-rightX);
            motorFrontLeft.setPower(rightX);
            motorBackRight.setPower(-rightX);
            motorBackLeft.setPower(rightX);
        } else if (Math.abs(leftY) >= 0.1 || Math.abs(leftX) >= 0.1) {
            if (Math.abs(leftY) >= Math.abs(leftX)) {
                //Move Y
                if (Math.abs(leftY) < 0.65) {
                    leftY /= 2;
                }
                motorFrontRight.setPower(leftY);
                motorFrontLeft.setPower(leftY);
                motorBackRight.setPower(leftY);
                motorBackLeft.setPower(leftY);
            } else {
                //Move X
                if (Math.abs(leftX) < 0.65) {
                    leftX /= 2;
                }
                if (gamepad1.right_bumper) {
                    leftX += (0.15 * (leftX / Math.abs(leftX)));
                }
                //motorFrontRight.setPower(-leftX);
                //motorFrontLeft.setPower(leftX);
                //motorBackRight.setPower(leftX);
                //motorBackLeft.setPower(-leftX);

                motorFrontRight.setPower(-leftX*0.6);
                motorFrontLeft.setPower(leftX*0.6);//-
                motorBackRight.setPower(leftX);
                motorBackLeft.setPower(-leftX);//-
            }
        } else {
            motorFrontRight.setPower(0);
            motorFrontLeft.setPower(0);
            motorBackRight.setPower(0);
            motorBackLeft.setPower(0);
        }




        //Intake (a intake (and stop) b outtake) d pad down is down; up is up, left or right to make horizontal
        if (gamepad2.a) {
            motorIntakeRight.setPower(0.35);
            motorIntakeLeft.setPower(-0.35);
        } else if (gamepad2.b) {
            motorIntakeRight.setPower(-0.35);
            motorIntakeLeft.setPower(0.35);
        } else if (gamepad2.left_trigger > 0.5){
            motorIntakeLeft.setPower(1);
            motorIntakeRight.setPower(1);
        } else {
            motorIntakeRight.setPower(0);
            motorIntakeLeft.setPower(0);
        }

        /*
        if (gamepad2.dpad_left || gamepad2.dpad_right) {
            servoIntake.setPosition(0.5);
        } else if (gamepad2.dpad_up) {
            servoIntake.setPosition(1);
        } else if (gamepad2.dpad_down) {
            servoIntake.setPosition(0.36);
        }
         */

        //Outtake; left joystick up and down; >0.7 to swing servo x axis; grabby servo: x to grab y to release
        if (Math.abs(leftY2) > 0.05) {
            if (leftY2 > 0 || motorOuttake.getCurrentPosition() <= 0) {
                motorOuttake.setPower(leftY2);
            }
        } else {
            if (motorOuttake.getCurrentPosition() < -100) {
                motorOuttake.setPower(0.1);
            } else {
                motorOuttake.setPower(0);
            }
        }

        if (leftX2 > 0.7) {
            servoOuttakeRotate.setPosition(0.78);
        } else if (leftX2 < -0.7) {
            servoOuttakeRotate.setPosition(0.06);
        }

        if (gamepad2.x) {
            servoOuttakeClaw.setPosition(0.45);
        } else if (gamepad2.y) {
            servoOuttakeClaw.setPosition(0);
        }

        if (gamepad2.right_bumper) {
            //Release Foundation
            servoFoundationRight.setPosition(0.75);
            servoFoundationLeft.setPosition(0.25);
        } else if (gamepad2.left_bumper) {
            //Grab Foundation
            servoFoundationRight.setPosition(0);
            servoFoundationLeft.setPosition(1);
        }
    }
}
