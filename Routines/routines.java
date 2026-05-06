package.org.firstinspires.ftc;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

private DcMotor intakeMotor1;

public class routines extends LinearOpMode {
    intakeMotor1 = hardwareMap.DcMotor.get(DcMotor.class, "intakeMotor1");
    
    public static void intake(){
        intakeMotor1.setPower(1);
    }
}