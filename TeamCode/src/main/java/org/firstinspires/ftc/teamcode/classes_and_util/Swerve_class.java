package org.firstinspires.ftc.teamcode.classes_and_util;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Swerve_class {

    AnalogInput pod_encoder;
    CRServo Swervo;
    DcMotor Drive_motor;
    IMU imu;
    int offset;
    int pod_num;

    public Swerve_class(AnalogInput nanoEncoder1, CRServo pod1Servo, DcMotor pod1Motor, int i) {
    }


    public double[] Servo_path(double x, double y, double rx, boolean lock) {
        double target_angle = Math.toDegrees(Math.atan2(x, y));
        int wheel_direction;

        double current_angle = pod_encoder.getVoltage() / 3.2 * 360.0;
        current_angle = (current_angle - offset) % 360;  // whatever call for the wheel pod pos optimize w lynx also make sure it gets its own actual pod wtv
        double turning_angle = 0;
        int multiplier = (target_angle == 0 && x == 0) ? 2:1;
        if (pod_num == 0) {multiplier=2;}
        target_angle =  (target_angle + ((((-45 + 90* pod_num)%360)*Math.signum(rx)) * Math.signum(rx)))/((double) 2 /multiplier);
        target_angle -= imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES); // optimize w lynx
        // idea is that we adjust target angle to be the opposite of whatever the target is
        // and reverse wheel direction and use that diff target
        if (lock) { target_angle = 135+(90*pod_num); }
        pod_num *= (int) rx;

        // This covers cases where the target angle i s actually within 90 degrees either way
        if (Math.abs(target_angle - current_angle) >= 270) {
            wheel_direction = 1;
            int var_name = (target_angle - current_angle > 0) ? -1 : 1;
            turning_angle = target_angle + (360 * var_name); // -1 or 1 // *1 or -1 if its positive 270 target - current, its -1 vice versa
            return new double[]{turning_angle, wheel_direction}; // and wheel dir
        }
        else if (Math.abs(target_angle - current_angle) <= 90) {
            wheel_direction = 1;
            turning_angle = target_angle - current_angle;
            return new double[]{turning_angle, wheel_direction}; // and wheel dir
        }

        if (target_angle <=180) {target_angle += 180;} else {target_angle -= 180;} //goofy ass looping logic im too lazy to find a cool math thing
        wheel_direction = -1;
        turning_angle = target_angle - current_angle;

        return new double[]{turning_angle, wheel_direction}; // and wheel dir
    }



}
