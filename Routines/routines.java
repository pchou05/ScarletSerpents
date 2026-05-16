package org.firstinspires.ftc.teamcode;



    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.hardware.CRServo;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.util.ElapsedTime;



public class routines extends LinearOpMode {

    double pastPosition = 0;
    private ElapsedTime timer = new ElapsedTime();
    private double currentRPM = 0;
    private DcMotor feederMotor;
    private DcMotor shooterMotor;
    // private CRServo Servo1;
    // private CRServo Servo2;
    private DcMotor intakeMotor1;
    // private DcMotor intakeMotor2;
    private String shooterState = "idle";
    private double offset = 0;
    
    private PIDController shooterPID;
    private boolean useRPMControl = true;
  
    
    public void intake(){
        intakeMotor1.setPower(1);
    }

    public void runOpMode() throws InterruptedException {
        shooterMotor = hardwareMap.dcMotor.get("shooterMotor");
        feederMotor = hardwareMap.dcMotor.get("feederMotor");
        // Servo1 = hardwareMap.crservo.get("Servo1");
        // Servo2 = hardwareMap.crservo.get("Servo2"); 
        // intakeMotor1 = hardwareMap.dcMotor.get("intakeMotor1");
        // intakeMotor2 = hardwareMap.dcMotor.get("intakeMotor2");
        
        // Initialize PID controller with tuned gains
        shooterPID = new PIDController(0.079, 0.0001, 0.001);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            updateRPM();
            
            // Toggle RPM control mode with dpad
            if(gamepad1.dpad_up) {
                useRPMControl = true;
                shooterPID.reset();
            } else if(gamepad1.dpad_down) {
                useRPMControl = false;
            }

            // Shooter state control
            if(gamepad1.y) {
                shooterState = "off";
            } else if (gamepad1.b) {
                shooterState = "far";
            } else if (gamepad1.x) {
                shooterState = "line";
            } else if (gamepad1.a) {
                shooterState = "corner";
            } else if (gamepad1.dpad_left) {
                shooterState = "idle";
            }
            
            handleShooter(shooterState);  

            
            
            // Offset adjustment - Trim for shot power
            if(gamepad2.right_bumper) {
                offset += 0.001;
            } else if(gamepad2.left_bumper) {
                offset -= 0.001;
            }
            
            // Display telemetry
            telemetry.addLine("=== SHOOTER INFO ===");
            telemetry.addData("Control Mode", useRPMControl ? "RPM CONTROL" : "POWER CONTROL");
            telemetry.addData("Shooter State", shooterState);
            telemetry.addData("Shooter RPM", "%.0f", currentRPM);
            telemetry.addData("Target RPM", "%.0f", getTargetRPM(shooterState));
            telemetry.addData("RPM Error", "%.0f", getTargetRPM(shooterState) - currentRPM);
            telemetry.addData("Shooter Power", "%.2f", shooterMotor.getPower());
            telemetry.addData("Power Offset (Trim)", "%.3f", offset);
            telemetry.addData("Shooter Encoder", shooterMotor.getCurrentPosition());
            
        
            telemetry.addLine("\n=== CONTROLS ===");
            telemetry.addData("DPad Up", "RPM Control Mode");
            telemetry.addData("DPad Down", "Power Control Mode");
            telemetry.addData("DPad Left", "Idle (300 RPM)");
            telemetry.addData("A", "Corner Shot");
            telemetry.addData("B", "Far Shot");
            telemetry.addData("X", "Line Shot");
            telemetry.addData("Y", "Off");
            telemetry.addData("GP2 Bumpers", "Adjust Offset");
            
            telemetry.update();
        }
    }
    private void updateRPM(){
        double elapsedTime = timer.seconds();
        double currentPosition = shooterMotor.getCurrentPosition();
        if (elapsedTime > 0.1){
            currentRPM = ((currentPosition - pastPosition) * 60 / 28) / elapsedTime;
            pastPosition = currentPosition;
            timer.reset();
        }
    }
    
    private double getRPM(){
        return currentRPM;
    }
    
    private double getTargetRPM(String state) {
        switch (state){
            case "line": return 3500;      // Medium distance shot
            case "far": return 4200;       // Far distance shot - highest RPM
            case "corner": return 3200;    // Corner shot - slightly lower
            case "idle": return 300;       // Idle/standby mode
            case "off": return 0;          // Motor off
            case "wall": return 0;         // Motor off
            default: return 0;
        }
    }
    
    private void setShooterRPM(double targetRPM){
        double currentRPM = getRPM();
        double error = targetRPM - currentRPM;
        
        double pidOutput = shooterPID.calculate(error);
        double motorPower = pidOutput + offset;
        
        motorPower = Math.max(0, Math.min(1, motorPower));
        
        shooterMotor.setPower(motorPower);
    }
    
    private void setShooterPower(double power) {
        shooterMotor.setPower(Math.max(0, Math.min(1, power + offset)));
    }
    
    public void hitRPM(double value){
        double rpm = getRPM();
        if (rpm > value + 500){
            shooterMotor.setPower(0.2);
        } else if(rpm < value - 500){
            shooterMotor.setPower(1);
        } else {
            shooterMotor.setPower(0.51);
        }
    }
    
    private void handleShooter(String state){
        if (useRPMControl) {
            double targetRPM = getTargetRPM(state);
            setShooterRPM(targetRPM);
        } else {
            switch (state){
                case "line":
                    setShooterPower(0.69);
                    break;
                    
                case "far":
                    setShooterPower(0.76);
                    break;
            
                case "corner":
                    setShooterPower(0.67);
                    break;
                    
                case "idle":
                    hitRPM(300);
                    break;
                    
                case "off":
                case "wall": 
                    setShooterPower(0);
                    break;
            }
        }
    }
    
    private class PIDController {
        private double kp;
        private double ki;
        private double kd;
        private double integral;
        private double previousError;
        private ElapsedTime pidTimer;
        
        public PIDController(double kp, double ki, double kd) {
            this.kp = kp;
            this.ki = ki;
            this.kd = kd;
            this.integral = 0;
            this.previousError = 0;
            this.pidTimer = new ElapsedTime();
        }
        
        public double calculate(double error) {
            double dt = pidTimer.seconds();
            pidTimer.reset();
            
            // Prevent integral windup
            integral += error * dt;
            if (integral > 100) integral = 100;
            if (integral < -100) integral = -100;
            
            double derivative = (dt > 0) ? (error - previousError) / dt : 0;
            previousError = error;
            
            return (kp * error) + (ki * integral) + (kd * derivative);
        }
        
        public void reset() {
            integral = 0;
            previousError = 0;
            pidTimer.reset();
        }

    public void Intake()
    {
      
    }
    
    public void Shooter()
{
    double rpm = getRPM();
 
       if(gamepad2.x)
    {
        if(480<rpm && rpm<520){
           feederMotor.setPower(1);
        }
        else{
             hitRPM(500);
        }
    }
}
    }

    }
