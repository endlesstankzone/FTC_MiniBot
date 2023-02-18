package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;


@TeleOp

public class FieldCentricMecanumTeleOpFineControl extends LinearOpMode {
    
    private CRServo frontLeftMotor;
    private CRServo backLeftMotor;
    private CRServo frontRightMotor;
    private CRServo backRightMotor;
    private Gamepad currentGamepad1; 
    
    boolean intakeToggle = false;
    double power = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure you ID's match your configuration
        frontLeftMotor = hardwareMap.get(CRServo.class, "frontLeftMotor");
        backLeftMotor = hardwareMap.get(CRServo.class, "backLeftMotor");
        frontRightMotor = hardwareMap.get(CRServo.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(CRServo.class, "backRightMotor");
        
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        
        // Reverse the right side motors
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        
        // Retrieve the IMU from the hardware map
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technially this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws and exception 
        imu.initialize(parameters);
        
        waitForStart();
        
        if (isStopRequested()) return;
        
        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
            
            double y = -gamepad1.left_stick_y * power; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1 * power; // Counteract imperfect strafing 
            double rx = gamepad1.right_stick_x * power;
            
            // Read inverse IMU heading, as the IMU heading is CW positive
            double botHeading = -imu.getAngularOrientation().firstAngle;
            telemetry.addData("Bot Heading", (botHeading * (180 / 3.14)));
            
            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);
            
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            
            if (currentGamepad1.a && !previousGamepad1.a) {
                intakeToggle = !intakeToggle;
            }
            
            if (intakeToggle) {
                power = .5;
                telemetry.addData("Motor Power", power);
            } 
            else {
                power = 1;
                telemetry.addData("Motor Power", power);
            }
            telemetry.update();
        }
    }
}
