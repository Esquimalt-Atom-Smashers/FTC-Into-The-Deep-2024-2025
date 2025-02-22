package org.firstinspires.ftc.teamcode.opmodes.testandtuning;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
@TeleOp(name="Servo Test", group="z. Test")
public class ServoTest extends LinearOpMode {

    // Declare hardware variables
    private Servo servo;
    private Servo secServo;

    // Initialize a timer
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        // Initialize the hardware variables
        servo = hardwareMap.get(Servo.class, "specElbow");
        secServo = hardwareMap.get(Servo.class, "specClaw");

        // Wait for the game to start
        waitForStart();

        while (opModeIsActive()) {
            // Get the Y-axis input from the gamepad's left joystick
            double yInput = -gamepad1.left_stick_y;  // Invert the value if you want the control to be inverted
            double secInput = - gamepad1.right_stick_y;
            // Map the joystick value to a servo position (0.0 to 1.0)
            double servoPosition = (yInput + 1) / 2; // Mapping joystick from -1 to 1 to 0 to 1 range
            double secServoPos = (secInput + 1) / 2;
            // Set the servo position
            servo.setPosition(servoPosition);
            secServo.setPosition(secServoPos);
            // Send telemetry data to the driver station
            telemetry.addData("Servo Position", servoPosition);
            telemetry.addData("SecServo Pos", secServoPos);
            telemetry.update();

            // Add a small delay to prevent overloading the loop
            sleep(10);
        }
    }
}
