`
private DcMotor intakeMotor;

@Override
public void init() {

    intakeMotor = hardwareMap.get(DcMotor.class, "intake");
    intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    intakeMotor.setDirection(DcMotor.Direction.FORWARD); 
}

@Override
public void loop() {
    
    if (gamepad1.a) {
        intakeMotor.setPower(1.0);  
    } else if (gamepad1.b) {
        intakeMotor.setPower(-1.0); 
    } else {
        intakeMotor.setPower(0);   
    }

    telemetry.addData("Intake Power", intakeMotor.getPower());
    telemetry.update();
}
