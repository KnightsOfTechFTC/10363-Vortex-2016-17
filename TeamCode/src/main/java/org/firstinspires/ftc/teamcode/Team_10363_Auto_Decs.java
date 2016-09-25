package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

/**
 * Created by Lego5 on 9/25/2016.
 */
public class Team_10363_Auto_Decs {
    public DcMotor v_motor_left_drive;
    public DcMotor v_motor_right_drive;
    GyroSensor SensorGyro;

    public void init(HardwareMap ahwMap) {
        try{
            v_motor_left_drive=ahwMap.dcMotor.get("left_drive");
            v_motor_left_drive.setDirection(DcMotor.Direction.FORWARD);
            v_motor_left_drive.setPower(0);
            v_motor_left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            v_motor_left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        catch (Exception p_exception){
            v_motor_left_drive=null;
            DbgLog.msg(p_exception.getLocalizedMessage());
        }
        try {
            v_motor_right_drive=ahwMap.dcMotor.get("right_drive");
            v_motor_right_drive.setDirection(DcMotor.Direction.REVERSE);
            v_motor_right_drive.setPower(0);
            v_motor_right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            v_motor_right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        catch (Exception p_exception){
            v_motor_right_drive=null;
            DbgLog.msg(p_exception.getLocalizedMessage());
        }
        try {
            SensorGyro=ahwMap.gyroSensor.get("gyro");
        }
        catch (Exception p_exception){
            DbgLog.msg(p_exception.getLocalizedMessage());
            SensorGyro=null;
        }
        if (SensorGyro!=null){
            SensorGyro.calibrate();
            while(SensorGyro.isCalibrating()){
                try {
                    Thread.sleep(50);
                }
                catch (Exception p_exception){}
            }
        }


    }
    public void m_left_drive_power(float power){
        if (v_motor_left_drive!=null){
            float sendpower= Range.clip(power,-1,1);
            v_motor_left_drive.setPower(sendpower);
        }

    }
    public void m_right_drive_power(float power){
        if (v_motor_right_drive!=null){
            float sendpower=Range.clip(power,-1,1);
            v_motor_right_drive.setPower(sendpower);
        }
    }
    public void setDrivePower(float left_power, float right_power){
        if (Math.abs(left_power-right_power)<=.1){
            m_left_drive_power((left_power+right_power)/2);
            m_right_drive_power((left_power+right_power)/2);
        }
        else {
            m_left_drive_power(left_power);
            m_right_drive_power(right_power);
        }
    }
    public double adjspeed(double speedModifier, int deltaAngle){
        return speedModifier*Math.sin(Math.toRadians(deltaAngle));
    }
    public int a_gyro_heading(){
        if (SensorGyro!=null){
            return SensorGyro.getHeading();
        }
        else {return -36000;}
    }

}
