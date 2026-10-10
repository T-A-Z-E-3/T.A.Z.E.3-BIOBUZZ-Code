package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.seattlesolvers.solverslib.photon.PhotonCore;

import org.firstinspires.ftc.teamcode.classes_and_util.Swerve_class;

import java.util.List;

@TeleOp
public class Swerve_1_driver extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your IDs match your configuration
        CRServo pod_1_servo = hardwareMap.crservo.get("Swervo1");
        DcMotor pod_1_motor = hardwareMap.dcMotor.get("Swotor1");
        CRServo pod_2_servo = hardwareMap.crservo.get("Swervo2");
        DcMotor pod_2_motor = hardwareMap.dcMotor.get("Swotor2");
        CRServo pod_3_servo = hardwareMap.crservo.get("Swervo3");
        DcMotor pod_3_motor = hardwareMap.dcMotor.get("Swotor3");
        CRServo pod_4_servo = hardwareMap.crservo.get("Swervo4");
        DcMotor pod_4_motor = hardwareMap.dcMotor.get("Swotor4");

        AnalogInput nanoEncoder1 = hardwareMap.get(AnalogInput.class, "nanoEncoder1");
        AnalogInput nanoEncoder2 = hardwareMap.get(AnalogInput.class, "nanoEncoder2");
        AnalogInput nanoEncoder3 = hardwareMap.get(AnalogInput.class, "nanoEncoder3");
        AnalogInput nanoEncoder4 = hardwareMap.get(AnalogInput.class, "nanoEncoder4");

        double[][] turn_vectors = {{0.5,0.5},{-0.5,0.5},{-0.5,-0.5},{0.5,-0.5}};
        IMU imu;
        imu = hardwareMap.get(IMU.class, "imu");
        Swerve_class pod1 = new Swerve_class(nanoEncoder1, pod_1_servo, pod_1_motor, 99);
        Swerve_class pod2 = new Swerve_class(nanoEncoder2, pod_2_servo, pod_2_motor, 99);
        Swerve_class pod3 = new Swerve_class(nanoEncoder3, pod_3_servo, pod_3_motor, 99);
        Swerve_class pod4 = new Swerve_class(nanoEncoder4, pod_4_servo, pod_4_motor, 99);

        double[] pod_1_info;
        double[] pod_2_info;
        double[] pod_3_info;
        double[] pod_4_info;

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        // usb facing dir or some shi
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8); // Can be adjusted based on user preference - but raising this number further can cause issues
        PhotonCore.enable();
        // make swerve class with motor, servo, encoder. have servo path be a method I think
        // ima make a file, constants or team_constants then it has a class swerve pod and u create it w the motor encoder and servo, method is servo path/ turn the servo which is a combo of servo path currently then it calls setpower or smth so its a method like Swervepod.servo and it calls servo path then  and motor power,
        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
            PhotonCore.CONTROL_HUB.clearBulkCache();
            PhotonCore.EXPANSION_HUB.clearBulkCache();
            // Swerve coax code

            // Controller sticks
            double x = gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;
            boolean lock = gamepad1.a;
            // if lock then do this else
            pod_1_info = pod1.Servo_path(x, y, (int) (1 * Math.signum(rx)), lock);
            pod_2_info = pod2.Servo_path(x, y, (int) (2 * Math.signum(rx)), lock); // I dont think this works cus the angle you want for turning the other way is 180 from the other yk
            pod_3_info = pod3.Servo_path(x, y, (int) (3 * Math.signum(rx)), lock);
            pod_4_info = pod4.Servo_path(x, y, (int) (4 * Math.signum(rx)), lock);

            // vector math so we need current quadrant it points to, then some -y,x then add 0.5 to both x and y if turning = 1
            // take current angle 0-360 and x,y pod number, turning dir,

            // vector math I think done but replace 0 with the pod num

            //swerve writes
            pod_1_servo.setPower((pod_1_info[0]/90));
            pod_1_motor.setPower((((Math.abs(x+(turn_vectors[0][0]*rx))+Math.abs(y+(turn_vectors[0][1])*rx))/2) * pod_1_info[1]));
            pod_2_servo.setPower((pod_2_info[0]/90));
            pod_2_motor.setPower((((Math.abs(x+(turn_vectors[0][0]*rx))+Math.abs(y+(turn_vectors[0][1])*rx))/2) * pod_2_info[1]));
            pod_3_servo.setPower((pod_3_info[0]/90));
            pod_3_motor.setPower((((Math.abs(x+(turn_vectors[0][0]*rx))+Math.abs(y+(turn_vectors[0][1])*rx))/2) * pod_3_info[1]));
            pod_4_servo.setPower((pod_4_info[0]/90));
            pod_4_motor.setPower((((Math.abs(x+(turn_vectors[0][0]*rx))+Math.abs(y+(turn_vectors[0][1])*rx))/2) * pod_4_info[1]));




            // pod_1_power = Swerve_pod(x, y, pod, turn(its -1 or 1, depending which direction, power comes later)) / 90
        }
    }

    /* --- Motor Placement ---
             "F": Front; "B": Back
               1 --F-- 4
               |       |
               |       |
               2 --B-- 3
     */
    /*
     * Computes the shortest turning angle for a swerve pod given a target
     * direction (x, y) and its current heading.
     */

}