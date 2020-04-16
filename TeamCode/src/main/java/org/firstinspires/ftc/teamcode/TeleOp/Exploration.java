package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Exploration extends OpMode {

    private DcMotor motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight, motorOuttake, motorIntakeRight, motorIntakeLeft, motorIntakeMiddle;
    private Servo servoOuttakeRotate, servoOuttakeClaw, servoFoundationLeft, servoFoundationRight, servoCapstone;

    private double leftX, rightX, leftY, rightY, leftX2, leftY2;

    private double leftBumperTime = 0, rightBumperTime = 0;

    private boolean leftBumper = false, rightBumper = false;

    @Override
    public void init() {
        motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorFrontLeft = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");
        motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorOuttake = hardwareMap.get(DcMotor.class, "motorOuttake");
        motorIntakeLeft = hardwareMap.get(DcMotor.class, "motorIntakeLeft");
        motorIntakeRight = hardwareMap.get(DcMotor.class, "motorIntakeRight");
        motorIntakeMiddle = hardwareMap.get(DcMotor.class, "motorIntakeMiddle");

        servoCapstone = hardwareMap.get(Servo.class, "servoCapstone");
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
        motorIntakeMiddle.setDirection(DcMotor.Direction.REVERSE);

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
        motorOuttake.setPower(0);
        motorIntakeRight.setPower(0);
        motorIntakeLeft.setPower(0);
        motorIntakeMiddle.setPower(0);

        servoCapstone.setPosition(0.52);

        motorOuttake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorOuttake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorOuttake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        resetStartTime();
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
            leftX *= 0.7;
            rightX *= 0.7;
            leftY *= 0.7;
            rightY *= 0.7;
        }

        //Drivetrain
        if (Math.abs(leftX) >= 0.1) {
            //Turning
            motorFrontRight.setPower(-leftX);
            motorFrontLeft.setPower(leftX);
            motorBackRight.setPower(-leftX);
            motorBackLeft.setPower(leftX);
        } else if (Math.abs(rightY) >= 0.1 || Math.abs(rightX) >= 0.1) {
            if (Math.abs(rightY) >= Math.abs(rightX)) {
                //Move Y
                if (Math.abs(rightY) < 0.65) {
                    rightY /= 2;
                }
                motorFrontRight.setPower(rightY);
                motorFrontLeft.setPower(rightY);
                motorBackRight.setPower(rightY);
                motorBackLeft.setPower(rightY);
            } else {
                //Move X
                if (Math.abs(rightX) < 0.65) {
                    rightX /= 2;
                }
                if (gamepad1.right_bumper) {
                    rightX += (0.15 * (rightX / Math.abs(rightX)));
                }
                if(rightX>0){
                    motorFrontRight.setPower(-rightX);
                    motorFrontLeft.setPower(rightX);
                    motorBackRight.setPower(rightX);
                    motorBackLeft.setPower(-rightX);
                } else if(rightX<0){
                    motorFrontRight.setPower(-rightX);
                    motorFrontLeft.setPower(rightX);
                    motorBackRight.setPower(rightX);
                    motorBackLeft.setPower(-rightX);
                }
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
            motorIntakeMiddle.setPower(1);
        } else if (gamepad2.b) {
            motorIntakeRight.setPower(-0.35);
            motorIntakeLeft.setPower(0.35);
            motorIntakeMiddle.setPower(-1);
        } else if (gamepad2.left_trigger > 0.5) {
            motorIntakeLeft.setPower(1);
            motorIntakeRight.setPower(1);
        } else {
            motorIntakeRight.setPower(0);
            motorIntakeLeft.setPower(0);
            motorIntakeMiddle.setPower(0);
        }

        //Outtake
        if (Math.abs(leftY2) > 0.05) {
            motorOuttake.setPower(leftY2);
        } else {
            if (motorOuttake.getCurrentPosition() < -100) {
                motorOuttake.setPower(0.1);
            } else {
                motorOuttake.setPower(0);
            }
        }

        telemetry.addData("Position: ", motorOuttake.getCurrentPosition());
        telemetry.update();

        //Check if bumper was actually pressed and not held
        if(gamepad2.left_bumper) {
            if(leftBumperTime == 0 || getRuntime() - leftBumperTime > 0.5) {
                leftBumper = !leftBumper;
            }
            leftBumperTime = getRuntime();
        }

        //Action depending on state of left bumper
        if (leftBumper) {
            servoOuttakeRotate.setPosition(0.1);

        } else {
            servoOuttakeRotate.setPosition(0.8);
        }

        if (gamepad2.x) {
            servoOuttakeClaw.setPosition(0.45);
        } else if (gamepad2.y) {
            servoOuttakeClaw.setPosition(0);
        }

        if (gamepad2.dpad_up) {
            servoCapstone.setPosition(0.02);
        } else if (gamepad2.dpad_down) {
            servoCapstone.setPosition(0.52);
        }

        //Check if bumper was actually pressed and not held
        if(gamepad2.right_bumper) {
            if(rightBumperTime == 0 || getRuntime() - rightBumperTime > 0.5) {
                rightBumper = !rightBumper;
            }
            rightBumperTime = getRuntime();
        }

        //Action depending on state of left bumper
        if (gamepad1.left_trigger > 0.4) {
            servoFoundationRight.setPosition(0.8);
            servoFoundationLeft.setPosition(0.75);
        } else {
            servoFoundationRight.setPosition(1);
            servoFoundationLeft.setPosition(1);
        }
    }
}
