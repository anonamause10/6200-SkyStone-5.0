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
@TeleOp(name="datOne")

public class teleop extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FL = null;
    private DcMotor FR = null;
    private DcMotor BL = null;
    private DcMotor BR = null;
    private DcMotor UD = null;
    private Servo thatYes = null;
    private Servo thatNo = null;
    private boolean xPrev = false;
    private boolean yPrev = false;
    private boolean sPrev = false;
    private double armpower = 0.5;
    private boolean lbumpprev = false;
    private boolean rbumpprev = false;


    /*
     * Code to run ONCE when the driver hits INIT
     */

    @Override
    public void runOpMode() throws InterruptedException {
        FL = hardwareMap.get(DcMotor.class, "fL");
        FR = hardwareMap.get(DcMotor.class, "fR");
        BL = hardwareMap.get(DcMotor.class, "bL");
        BR = hardwareMap.get(DcMotor.class, "bR");
        FL.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.FORWARD);
        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        UD = hardwareMap.get(DcMotor.class, "uD");
        UD.setDirection(DcMotor.Direction.FORWARD);
        UD.setPower(0);
        UD.setTargetPosition(0);
        UD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        thatYes = hardwareMap.get(Servo.class, "servo");
        thatNo = hardwareMap.get(Servo.class, "servo2");
        thatYes.setPosition(.4);
        thatNo.setPosition(0);

        telemetry.addData("Robot", "Initialized");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            drive();

            if (gamepad1.a) {
                thatYes.setPosition(0);
                thatNo.setPosition(0.4);
            } else if (gamepad1.b) {
                thatYes.setPosition(0.4);
                thatNo.setPosition(0);
            }
            if (gamepad1.x && !xPrev) {
                UD.setTargetPosition(UD.getTargetPosition() - 7) ;
            } else if (gamepad1.y && !yPrev) {
                UD.setTargetPosition(UD.getTargetPosition() + 7);
            }
            if (gamepad1.left_bumper && !lbumpprev) {
                armpower -= 0.05;
            }
            if (gamepad1.right_bumper && !rbumpprev) {
                armpower += 0.05;
            }
            if (gamepad1.start && !sPrev){
                UD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                UD.setPower(armpower);
            }
            xPrev = gamepad1.x;
            yPrev = gamepad1.y;
            rbumpprev = gamepad1.right_bumper;
            lbumpprev = gamepad1.left_bumper;


            telemetry.addData("Wheel Power", "front left (%.2f), front right (%.2f), " +
                            "back left (%.2f), back right (%.2f)", FL.getPower(), FR.getPower(),
                    BL.getPower(), BR.getPower());
            telemetry.addData("Set Arm Power:", armpower);
            telemetry.addData("Arm Power:", UD.getPower());
            telemetry.addData("ArmPos", UD.getTargetPosition());
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
        if (gamepad1.right_trigger != 0 || gamepad1.left_trigger != 0) {
            v1 *= 2.85;
            v2 *= 2.85;
            v3 *= 2.85;
            v4 *= 2.85;
        }
        FL.setPower(v1 * 0.5);
        FR.setPower(v2 * 0.5);
        BL.setPower(v3 * 0.5);
        BR.setPower(v4 * 0.5);
        //OK YOU GOOD NOW
    }
}


