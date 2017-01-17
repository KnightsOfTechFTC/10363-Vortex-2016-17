package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Objects;

/**
 * Created by mathnerd on 1/16/17.
 */

public class ExperimentalVuforiaTestWithstraightDrive extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor v_motor_left_drive;
    DcMotor v_motor_right_drive;
    GyroSensor SensorGyro;
    ColorSensor FrontColor;
    @Override
    public void runOpMode() throws InterruptedException {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.vuforiaLicenseKey = "AWThhw//////AAAAGYgme8IEP0VXvW2eMc9GLHcCZt2HTjBY2BEZ7DmxzEgLDypsGvRgR2xr2douQ6h3nAzHpg7/HFpa4/DOlekbygKLhWdBAH2AhAu2r6nAn4ejWfQq32k4JVGOTbAMkx7H2fuHDYZduZQJiW/1pFJt0SdcqvClYOFtbdb+OaKHOTkLgmI3zWDBtjfM6Pc+FRchtsK3ITl1MxVtsVNsfZNC2UQREHd23ABZsQ0jrFcXaDwmR3Q1s3tOSRs3lMdJXk+riKmk2yLyat+pIRzHoUuvTQURKcvqgK00LVqWiOaarRlnOnccxzf2lO5jv4v2gohQXAxu6KpAQxsDyj1JrKYv91mWssJKeTbeXchIeLvqyCpn";
        parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.BUILDINGS;


        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);
        VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("wheels");
        beacons.get(1).setName("tools");
        beacons.get(2).setName("legos");
        beacons.get(3).setName("gears");



        try {
            v_motor_left_drive = hardwareMap.dcMotor.get("right_drive");
            v_motor_left_drive.setDirection(DcMotor.Direction.REVERSE);
            v_motor_left_drive.setPower(0);
            v_motor_left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            v_motor_left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        //If it doesn't work, set the motor to null and add record the problem in the Debug log.
        catch (Exception p_exception) {
            v_motor_left_drive = null;
            DbgLog.msg(p_exception.getLocalizedMessage());
        }
        //Same as above for the right motor, but reversed
        try {
            v_motor_right_drive = hardwareMap.dcMotor.get("left_drive");
            v_motor_right_drive.setDirection(DcMotor.Direction.FORWARD);
            v_motor_right_drive.setPower(0);
            v_motor_right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            v_motor_right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception p_exception) {
            v_motor_right_drive = null;
            DbgLog.msg(p_exception.getLocalizedMessage());
        }
        try {
            SensorGyro = hardwareMap.gyroSensor.get("gyro");
        } catch (Exception p_exception) {
            DbgLog.msg(p_exception.getLocalizedMessage());
            SensorGyro = null;
        }
        //Calibrate the gyro. Drive team- don't hit start until the light starts blinking!
        if (SensorGyro != null) {
            SensorGyro.calibrate();
            while (SensorGyro.isCalibrating()) {
                Thread.sleep(50);
            }
        }
        try {
            FrontColor = hardwareMap.colorSensor.get("front-color");
            FrontColor.setI2cAddress(I2cAddr.create8bit(0x46));
            FrontColor.enableLed(false);
        } catch (Exception p_exception) {
            DbgLog.msg(p_exception.getLocalizedMessage());
            FrontColor = null;
        }
        waitForStart();
        timedrive(1000,.3,.3,-5);
        gyroturn(45, 11);
        gyrohold(1000,45,2);
        beacons.activate();
        while (opModeIsActive()&&FrontColor.red()<2&&FrontColor.blue()<2) {
            double targetdeg=0;
            for (VuforiaTrackable beac : beacons){
                OpenGLMatrix pos=((VuforiaTrackableDefaultListener)beac.getListener()).getPose();
                if (pos!=null){
                    VectorF translation=pos.getTranslation();
                    telemetry.addData(beac.getName()+"translation", translation);
                    double degtoturn=Math.toDegrees(Math.atan2(translation.get(1),translation.get(2)));
                    telemetry.addData(beac.getName()+"degrees",degtoturn);
                    if (Objects.equals(beac.getName(), "gears")){
                        targetdeg=degtoturn;
                    }
                }
            }
            double adjspeed=Math.sin(((2*Math.PI)/360)*(-targetdeg));
            setDrivePower((float) (.3f-adjspeed),(float) (.3f+adjspeed));
            telemetry.addData("2: adjspeed: " ,adjspeed);
            telemetry.update();
        }

    }
    public void timedrive(int mills, double speedleft, double speedright, int targetheading) throws InterruptedException{
        runtime.reset();
        while (opModeIsActive() && runtime.milliseconds()<mills) {
            telemetry.addData("-1: time driving",runtime.milliseconds());
            if (targetheading>=0&&targetheading<=360){
                double adjspeed=(speedleft+speedright)*Math.sin(((2*Math.PI)/360)*(a_gyro_heading()-targetheading));
                telemetry.addData("2: adjspeed: " ,adjspeed);
                v_motor_left_drive.setPower(speedleft-adjspeed);
                v_motor_right_drive.setPower(speedright+adjspeed);
            }
            else{
                v_motor_left_drive.setPower(speedleft);
                v_motor_right_drive.setPower(speedright);
            }

            telemetry.addData("5: Heading ", a_gyro_heading());
            telemetry.update();

            // Allow time for other processes to run.
            idle();
        }
        setDrivePower(0,0);
    }
    public int a_gyro_heading(){
        if (SensorGyro!=null){
            return SensorGyro.getHeading();
        }
        else {return -36000;}
    }
    public void setDrivePower(float left_power, float right_power){
        m_left_drive_power(left_power);
        m_right_drive_power(right_power);

    }
    public void m_right_drive_power(float power){
        if (v_motor_right_drive!=null){
            float sendpower= Range.clip(power,-.6f,.6f);
            v_motor_right_drive.setPower(sendpower);
        }
    }
    public void m_left_drive_power(float power){
        if (v_motor_left_drive!=null){
            float sendpower= Range.clip(power,-.6f,.6f);
            v_motor_left_drive.setPower(sendpower);
        }
    }
    public void gyroturn(int targetheading, int error)throws InterruptedException{
        runtime.reset();
        double tempGyro=999999;
        while((tempGyro<targetheading-error||tempGyro>targetheading+error)&&opModeIsActive()){
            tempGyro=a_gyro_heading();
            double adjspeed=(2.2)*Math.sin(((2*Math.PI)/360)*(tempGyro-targetheading));
            telemetry.addData("2: adjspeed: " ,adjspeed);
            if (v_motor_left_drive!=null&&v_motor_right_drive!=null){
                v_motor_left_drive.setPower(Range.clip(-adjspeed,-1,1));
                v_motor_right_drive.setPower(Range.clip(adjspeed,-1,1));
            }
            telemetry.addData("0: target heading",targetheading);
            telemetry.addData("1: actual heading",tempGyro);
            telemetry.addData("3: time passed (ms)", runtime.milliseconds());
            telemetry.addData("4: error range", error);
            telemetry.update();
            // Allow time for other processes to run.

            idle();
        }
        setDrivePower(0,0);
    }
    public void gyrohold(int mills, int targetheading,double spmod)throws InterruptedException{
        runtime.reset();
        while (runtime.milliseconds()<mills&&opModeIsActive()){
            telemetry.addData("0: target heading",targetheading);
            telemetry.addData("1: actual heading",a_gyro_heading());
            double adjspeed=(spmod)*Math.sin(((2*Math.PI)/360)*(a_gyro_heading()-targetheading));
            telemetry.addData("2: adjspeed: " ,adjspeed);
            if (v_motor_left_drive!=null&&v_motor_right_drive!=null){
                v_motor_left_drive.setPower(Range.clip(-adjspeed,-1,1));
                v_motor_right_drive.setPower(Range.clip(adjspeed,-1,1));
            }
            telemetry.addData("3: time passed (ms)", runtime.milliseconds());
            telemetry.update();
            // Allow time for other processes to run.

            idle();
        }
    }

}
