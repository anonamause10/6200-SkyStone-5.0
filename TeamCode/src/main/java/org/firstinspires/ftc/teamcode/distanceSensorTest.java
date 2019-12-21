package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

@TeleOp(name = "distance sensor test")
public class distanceSensorTest extends LinearOpMode{
    private DistanceSensor sR = null;
    private DistanceSensor sR2 = null;
    private DistanceSensor sRF = null;
    private DistanceSensor sRL = null;
    private DistanceSensor sRR = null;
    private DcMotor FL = null;
    private DcMotor FR = null;
    private DcMotor BL = null;
    private DcMotor BR = null;
    private DcMotor LIFT = null;
    private DcMotor IN1 = null;
    private DcMotor IN2 = null;
    private ElapsedTime runtime = new ElapsedTime();
    BNO055IMU imu;
    @Override
    public void runOpMode() throws InterruptedException {
        sR = hardwareMap.get(DistanceSensor.class, "DSB");
        sR2 = hardwareMap.get(DistanceSensor.class, "DS2");
        sRF = hardwareMap.get(DistanceSensor.class, "DSF");
        sRL = hardwareMap.get(DistanceSensor.class, "DSL");
        sRR = hardwareMap.get(DistanceSensor.class, "DSR");
        FL = hardwareMap.get(DcMotor.class, "fL");FR = hardwareMap.get(DcMotor.class, "fR");
        BL = hardwareMap.get(DcMotor.class, "bL");BR = hardwareMap.get(DcMotor.class, "bR");
        FL.setDirection(REVERSE);FR.setDirection(FORWARD);
        BL.setDirection(REVERSE);BR.setDirection(FORWARD);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setPower(0);FR.setPower(0);
        BL.setPower(0);BR.setPower(0);
        IN1 = hardwareMap.get(DcMotor.class, "IN1");
        IN1.setDirection(FORWARD);
        IN2 = hardwareMap.get(DcMotor.class, "IN2");
        IN2.setDirection(REVERSE);

        LIFT = hardwareMap.get(DcMotor.class, "LIFT");
        LIFT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LIFT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LIFT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LIFT.setPower(0);
        LIFT.setTargetPosition(LIFT.getCurrentPosition());
        initIMU();
        runtime.reset();
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }
        while(opModeIsActive()){
            if(gamepad1.left_bumper){
                IN1.setPower(-0.4);
                IN2.setPower(-0.4);
            }else if(gamepad1.right_bumper && sR2.getDistance(DistanceUnit.MM)>70){
                IN1.setPower(0.7);
                IN2.setPower(0.7);
            }else{
                IN1.setPower(0);
                IN2.setPower(0);
            }

            if(gamepad1.left_trigger != 0){
                IN1.setPower(-gamepad1.left_trigger);
                IN2.setPower(-gamepad1.left_trigger);
            }else if(gamepad1.right_trigger != 0){
                IN1.setPower(gamepad1.right_trigger);
                IN2.setPower(gamepad1.right_trigger);
            }
            drive();
            telemetry.addData("Wheel Position", "front left (%.1f), front right (%.1f), " +
                            "back left (%.1f), back right (%.1f)", (float)FL.getCurrentPosition(), (float)FR.getCurrentPosition(),
                    (float)BL.getCurrentPosition(), (float)BR.getCurrentPosition());
            telemetry.addData("backSensor", sR.getDistance(DistanceUnit.MM));
            telemetry.addData("leftSensor", sRL.getDistance(DistanceUnit.MM));
            telemetry.addData("rightSensor", sRR.getDistance(DistanceUnit.MM));
            telemetry.addData("frontSensor", sRF.getDistance(DistanceUnit.MM));
            telemetry.addData("intakeSensor", sR2.getDistance(DistanceUnit.MM));
            telemetry.addData("LIFT position", LIFT.getCurrentPosition());
            telemetry.addData("IMU", getHeading());
            telemetry.addData("Runtime:", runtime.seconds());
            telemetry.update();
        }
    }
    private void drive(){

        double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = Math.pow(gamepad1.right_stick_x,4);
        if(gamepad1.right_stick_x<0){
            rightX *= -1;
        }
        double v1 = r * Math.cos(robotAngle) + rightX;
        double v2 = r * Math.sin(robotAngle) - rightX;
        double v3 = r * Math.sin(robotAngle) + rightX;
        double v4 = r * Math.cos(robotAngle) - rightX;
        if (gamepad1.x) {
            v1 *= 2;
            v2 *= 2;
            v3 *= 2;
            v4 *= 2;
        }
        if(gamepad1.left_stick_button || gamepad1.right_stick_button){
            v1 *= .5;
            v2 *= .5;
            v3 *= .5;
            v4 *= .5;
        }
        FL.setPower(v1*0.5);
        FR.setPower(v2*0.5);
        BL.setPower(v3*0.5);
        BR.setPower(v4*0.5);
    }

    private void initIMU(){
        BNO055IMU.Parameters parameters2 = new BNO055IMU.Parameters();
        parameters2.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters2.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters2.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters2.loggingEnabled = true;
        parameters2.loggingTag = "IMU";
        parameters2.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters2);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }
    public double getHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (angles.firstAngle+360)%360;
    }
}
