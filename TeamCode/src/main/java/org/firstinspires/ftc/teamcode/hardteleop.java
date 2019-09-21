package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by isong on 10/17/18.
 */
@TeleOp(name="Hard")

public class hardteleop extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FL = null;
    private DcMotor FR = null;
    private DcMotor BL = null;
    private DcMotor BR = null;
    private DcMotor UD = null;
    private Servo thatYes = null;
    private ElapsedTime servotime = new ElapsedTime();


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void runOpMode() throws InterruptedException {
        FL = hardwareMap.get(DcMotor.class, "fL");FR = hardwareMap.get(DcMotor.class, "fR");
        BL = hardwareMap.get(DcMotor.class, "bL");BR = hardwareMap.get(DcMotor.class, "bR");
        FL.setDirection(DcMotor.Direction.FORWARD);BL.setDirection(DcMotor.Direction.FORWARD);
        FR.setDirection(DcMotor.Direction.REVERSE);BR.setDirection(DcMotor.Direction.REVERSE);
        FL.setPower(0);FR.setPower(0);BL.setPower(0);BR.setPower(0);
        UD = hardwareMap.get(DcMotor.class, "uD");
        UD.setDirection(DcMotor.Direction.FORWARD);
        UD.setPower(0);
        thatYes = hardwareMap.get(Servo.class, "servo");
        thatYes.setPosition(.4);


        telemetry.addData("Robot", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            drive();

            if(gamepad1.a){
                thatYes.setPosition(0);
            }
            if(servotime.seconds()>0.6){
                thatYes.setPosition(0.4);
                servotime.reset();
            }
            if(gamepad1.x){
                UD.setPower(1);
            }else if(gamepad1.y){
                UD.setPower(-1);
            }else{
                UD.setPower(0);
            }


            telemetry.addData("Wheel Power", "front left (%.2f), front right (%.2f), " +
                            "back left (%.2f), back right (%.2f)", FL.getPower(), FR.getPower(),
                    BL.getPower(), BR.getPower());
            telemetry.addData("Linear Actuator Power:", UD.getPower());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }

    private void drive(){
        //DONT TOUCH THIS

        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        double v1 = r * Math.cos(robotAngle) + rightX;
        double v2 = r * Math.sin(robotAngle) - rightX;
        double v3 = r * Math.sin(robotAngle) + rightX;
        double v4 = r * Math.cos(robotAngle) - rightX;
        if(gamepad1.right_bumper||gamepad1.left_bumper||gamepad1.right_trigger!=0||gamepad1.left_trigger!=0) {
            v1 *=2.85;
            v2 *=2.85;
            v3 *=2.85;
            v4 *=2.85;
        }
        FL.setPower(v1);
        FR.setPower(v2);
        BL.setPower(v3);
        BR.setPower(v4);
        //OK YOU GOOD NOW
    }
}
