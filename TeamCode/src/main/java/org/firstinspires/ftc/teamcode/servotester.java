package org.firstinspires.ftc.teamcode;

import android.content.Context;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.ftccommon.SoundPlayer;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
/**
 * Created by isong on 10/17/18.
 */
@TeleOp(name="one motor")

public class servotester extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor S1 = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */

    @Override
    public void runOpMode() throws InterruptedException {
        S1 = hardwareMap.get(DcMotor.class, "LIFT");
        S1.setPower(0);
        S1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        S1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Robot", "Initialized");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        //waitForStart();
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }
        runtime.reset();
        while (opModeIsActive()) {


            S1.setPower(-gamepad1.left_stick_y);

            telemetry.addData("Power:", S1.getPower());
            telemetry.addData("Position:", S1.getCurrentPosition());
            telemetry.addData("Time", runtime.seconds());
            telemetry.update();
        }
    }
}
