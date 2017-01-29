
package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
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


@Autonomous(name = "vuforia")
public class VuforiaOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor v_motor_right_drive;
        DcMotor v_motor_left_drive;
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = "AWThhw//////AAAAGYgme8IEP0VXvW2eMc9GLHcCZt2HTjBY2BEZ7DmxzEgLDypsGvRgR2xr2douQ6h3nAzHpg7/HFpa4/DOlekbygKLhWdBAH2AhAu2r6nAn4ejWfQq32k4JVGOTbAMkx7H2fuHDYZduZQJiW/1pFJt0SdcqvClYOFtbdb+OaKHOTkLgmI3zWDBtjfM6Pc+FRchtsK3ITl1MxVtsVNsfZNC2UQREHd23ABZsQ0jrFcXaDwmR3Q1s3tOSRs3lMdJXk+riKmk2yLyat+pIRzHoUuvTQURKcvqgK00LVqWiOaarRlnOnccxzf2lO5jv4v2gohQXAxu6KpAQxsDyj1JrKYv91mWssJKeTbeXchIeLvqyCpn";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(params);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Lego");
        beacons.get(3).setName("Gears");
        try{
            v_motor_left_drive= hardwareMap.dcMotor.get("left_drive");
            v_motor_left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            v_motor_left_drive.setDirection(DcMotorSimple.Direction.FORWARD);
            v_motor_left_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            v_motor_left_drive.setPower(0);
        }catch (Exception p_exception){
            v_motor_left_drive=null;
            DbgLog.msg(p_exception.getLocalizedMessage());
        }
        try{
            v_motor_right_drive= hardwareMap.dcMotor.get("right_drive");
            v_motor_right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            v_motor_right_drive.setDirection(DcMotorSimple.Direction.REVERSE);
            v_motor_right_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            v_motor_right_drive.setPower(0);
        }catch (Exception p_exception){
            v_motor_right_drive=null;
            DbgLog.msg(p_exception.getLocalizedMessage());
        }
        telemetry.addData("ready",79);
        telemetry.update();
        waitForStart();
        ElapsedTime runtime=new ElapsedTime();
        runtime.reset();
        if (v_motor_left_drive!=null&&v_motor_right_drive!=null){
            v_motor_left_drive.setPower(.3);
            v_motor_right_drive.setPower(.3);
            while (runtime.seconds()<1&&opModeIsActive()){
                telemetry.addData("1:moving forward",1);
                telemetry.addData("runtime",runtime.milliseconds());
                telemetry.update();
            }
            runtime.reset();
            v_motor_left_drive.setPower(0);
            v_motor_right_drive.setPower(0);
            v_motor_left_drive.setPower(-1);
            v_motor_right_drive.setPower(1);
            while (runtime.seconds()<.8&&opModeIsActive()){
                telemetry.addData("2:turning",2);
                telemetry.addData("runtime",runtime.milliseconds());
                telemetry.update();
            }
            v_motor_left_drive.setPower(0);
            v_motor_right_drive.setPower(0);
        }
        beacons.activate();
        runtime.reset();
        while (runtime.seconds()<1){}
        while (opModeIsActive()){
            double deg=0;
            for(VuforiaTrackable beac : beacons){
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beac.getListener()).getPose();

                if(pose != null){
                    VectorF translation = pose.getTranslation();

                    telemetry.addData(beac.getName() + "-Translation", translation);

                    double degreesToTurn = Math.toDegrees(Math.atan2(translation.get(1), translation.get(2)));
                    if (degreesToTurn<0){
                        degreesToTurn+=180;
                    }else{
                        degreesToTurn-=180;
                    }
                    degreesToTurn*=-1;
                    telemetry.addData(beac.getName() + "-Degrees", degreesToTurn);
                    if (Objects.equals(beac.getName(), "Wheels")){
                        deg=degreesToTurn;
                    }
                }
            }
            telemetry.update();
            double adjspeed=(1)*Math.sin(((2*Math.PI)/360)*(deg));
            if (v_motor_left_drive!=null&&v_motor_right_drive!=null){
            v_motor_left_drive.setPower(.4-adjspeed);
            v_motor_right_drive.setPower(.4+adjspeed);
        }
        }
    }
}