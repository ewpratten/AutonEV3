
// Path definition
class Path {
    private scale: number;

    public waypoints: Pose[];




    public Path() {

        // Create list of waypoints


    }
}

// Constants
let left_motor_port = "a";
let right_motor_port = "b";
let gyro_port = 1;
let wheel_diameter = 0.04; // TODO add real wheel diameter
let wheel_circumference = wheel_diameter * Math.PI;
let RampRate = 0.0;

/* Logging */
let max_log_lines = 12;
let active_log_messages = ["", ""];

function log(message: string) {
    active_log_messages.unshift(message);

    if (active_log_messages.length > max_log_lines) {
        active_log_messages.pop();
    }

    for (let i = 0; i < active_log_messages.length - 1; i++) {

        // Disp the message to the screen
        brick.showString(active_log_messages[i], max_log_lines - i);

    }
}




/* ########## MathUtils ########## */

/**
 * Convert degrees to radians
 * @param degrees Degrees
 */
function d2r(degrees: number) {
    let pi = Math.PI;
    return degrees * (pi / 180);
}

/**
 * Convert radians to degrees
 * @param radians Radians
 */
function r2d(radians: number) {
    let pi = Math.PI;
    return radians * (180 / pi);
}

function hypot(x: number, y: number) {

    let max = 0;
    let s = 0;
    let arg = Math.abs(y);
    if (arg > max) {
        s *= (max / arg) * (max / arg);
        max = arg;
    }
    s += arg === 0 && max === 0 ? 0 : (arg / max) * (arg / max);

    return max === 1 / 0 ? 1 / 0 : max * Math.sqrt(s);
}

function getWrappedError(currentAngle: number, desiredAngle: number) {
    let phi: number = Math.abs(currentAngle - desiredAngle) % 360; // This is either the distance or 360 - distance
    let distance: number = phi > 180 ? 360 - phi : phi;

    // Determine the sign (is the difference positive of negative)
    let sign: number = (currentAngle - desiredAngle >= 0 && currentAngle - desiredAngle <= 180)
        || (currentAngle - desiredAngle <= -180 && currentAngle - desiredAngle >= -360) ? 1 : -1;

    // Return the final difference
    return distance * sign;

}

/**
 * Returns value clamped between low and high boundaries.
 *
 * @param value Value to clamp.
 * @param min   The lower boundary to which to clamp value.
 * @param max  The higher boundary to which to clamp value.
 */
function clamp(val: number, min: number, max: number) {
    return Math.max(min, Math.min(val, max));
}

/**
 * Format a float to 2 decimals
 * @param val Value to format
 */
function pf2(val: number): number {
    return Math.round(val * 100) / 100;
}

/* ########## Gyro Class ########## */

class Gyro {

    public gyro_port: string;

    public getDegrees() {
        return sensors.gyro1.angle();
    }

    public getRotation(): Rotation {
        return new Rotation(d2r(this.getDegrees()));
    }



}


/* ########## Kinematics ########## */

class DriveSignal {
    public left_goal: number;
    public right_goal: number;

    /**
     * 
     * @param left left goal value
     * @param right right goal value
     */
    public DriveSignal(left: number, right: number) {
        this.left_goal = left;
        this.right_goal = right;
    }
}



/* ########## Localization ########## */

class Rotation {
    private rads: number;

    /**
     * 
     * @param rads Angle in radians
     */
    constructor(rads: number) {
        this.rads = rads;
    }

    public getDegrees() {
        return r2d(this.rads);
    }

    public getRadians() {
        return this.rads;
    }

    public setRadians(rads: number) {
        this.rads = rads;
    }

    public fromPoint(x: number, y: number): Rotation {
        let mag: number = hypot(x, y);
        let sin: number;
        let cos: number;
        if (mag > 1e-6) {
            sin = y / mag;
            cos = x / mag;
        } else {
            sin = 0;
            cos = 1;
        }

        return new Rotation(Math.atan2(sin, cos));

    }

    public rotateBy(other: Rotation): Rotation {
        let x: number = Math.cos(this.rads) * Math.cos(other.getRadians()) - Math.sin(this.rads) * Math.sin(other.getRadians());
        let y: number = Math.cos(this.rads) * Math.sin(other.getRadians()) - Math.sin(this.rads) * Math.cos(other.getRadians());

        return this.fromPoint(x, y);
    }

    public unaryMinus(): Rotation {
        return new Rotation(this.rads * -1);
    }

    public minus(other: Rotation): Rotation {
        return this.rotateBy(other.unaryMinus());
    }
}

/**
 * Create a rotation using degrees
 * @param degrees Degree angle
 */
function createRotationDegrees(degrees: number): Rotation {
    return new Rotation(r2d(degrees));
}

/**
 * A position in 2D space
 */
class Pose {
    public x: number;
    public y: number;
    public theta: Rotation;

    /**
     * Create a pose in 2D space
     * @param x X component
     * @param y Y component
     * @param theta Angle
     */
    constructor(x: number, y: number, theta: Rotation = new Rotation(0.0)) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    public toString(): string {
        return "Pose<" + pf2(this.x) + ", " + pf2(this.y) + ", " + pf2(this.theta.getDegrees()) + ">"
    }
}


/**
 * Create a pose that follows a scaling factor
 * @param x  X component
 * @param y  Y component
 * @param theta  Angle in degrees
 * @param scale the scaling multiplier
 * @return returns a scaled posed
 */
function createScaledPose(x: number, y: number, theta: number, scale: number): Pose {
    return new Pose(x * scale, y * scale, createRotationDegrees(theta));
}

/**
 * A tool for localizing a differential drivebase
 */
class Localizer {

    private gyroOffset: Rotation;
    private prevAngle: Rotation;

    private pose: Pose;
    private prevDistLeft: number;
    private prevDistRight: number;

    constructor(startAngle: Rotation, startPose: Pose) {
        this.pose = startPose;
        this.gyroOffset = startAngle.minus(startPose.theta);
        this.prevAngle = startPose.theta;

        this.prevDistLeft = 0.0;
        this.prevDistRight = 0.0;
    }

    public reset(pose: Pose, angle: Rotation) {
        this.pose = pose;
        this.prevAngle = pose.theta;
        this.gyroOffset = pose.theta.minus(angle);

        this.prevDistLeft = 0.0;
        this.prevDistRight = 0.0;
    }

    public update(currentAngle: Rotation, leftMeters: number, rightMeters: number) {
        let deltaLeft: number = leftMeters;//- this.prevDistLeft;
        let deltaRight: number = rightMeters;// - this.prevDistRight;

        this.prevDistLeft = leftMeters;
        this.prevDistRight = rightMeters;

        let deltaPose: number = (deltaLeft + deltaRight) / 2.0;
        let angle: Rotation = currentAngle.rotateBy(new Rotation(getWrappedError(this.gyroOffset.getRadians(), 0)));

        // TODO: These may need to be flipped cos/sin
        let newX: number = this.pose.x + (deltaPose * Math.sin(angle.getRadians()));
        let newY: number = this.pose.y + (deltaPose * Math.cos(angle.getRadians()));

        this.pose.x = newX;
        this.pose.y = newY;
        this.pose.theta = angle;

    }

    public getPoseMeters(): Pose {
        return this.pose;
    }
}

/* ########## Drive code ########## */

class Motor {

    private motor: motors.Motor;
    private output: number;

    private rps: number;
    private lastAngle: number;
    private lastTime: number;

    constructor(motor: motors.Motor) {
        this.motor = motor;
    }

    public set(output: number) {
        this.motor.run(output * 100);
        this.output = output;
    }

    public setBrakes(on: boolean) {
        this.motor.setBrake(on);
    }

    public update() {
        let angle: number = this.motor.angle();
        let dtheta: number = getWrappedError(angle, this.lastAngle);
        this.lastAngle = angle;
        let time: number = control.millis() / 1000;
        let dt: number = time - this.lastTime;
        this.lastTime = time;
        this.rps = dtheta * dt;
    }

    public getRPS(): number {
        return this.rps;

    }

    public getRPM(): number {
        return this.rps / 60;
    }
}

// Motor defs
let leftMotor: Motor = new Motor(motors.largeA);
let rightMotor: Motor = new Motor(motors.largeB);

function arcadeDrive(speed: number, turn: number) {
    let left: number = speed + turn;
    let right: number = speed - turn;

    let magnitude: number = Math.max(left, right);
    if (magnitude > 1.0) {
        left /= magnitude;
        right /= magnitude;
    }

    // Send tank command
    leftMotor.set(left);
    rightMotor.set(right);
}


/* ########## Main Robot Code ########## */

let gyro: Gyro = new Gyro();
let localizer: Localizer = new Localizer(gyro.getRotation(), new Pose(0, 0, createRotationDegrees(0.0)));

function init() {
    log("Robot code starting...");
    log("Autonomous car");

    log("Setting motor modes");
    leftMotor.setBrakes(true);
    rightMotor.setBrakes(true);

    log("Awaiting button press");

}

let canRunCode = false;

/**
 * All main code shall be run from here to reduce issues with cross-compiling to "blocks mode"
 */
function loop() {

    // Update both motors
    leftMotor.update();
    rightMotor.update();

    // Wait for button press to run code
    if (!canRunCode) {
        canRunCode = brick.buttonDown.isPressed();
        if (canRunCode) {
            log("Running program");
        }
        return;
    }

    // Get the robot's current position
    let robotPose: Pose = handleLocalization();

    log(robotPose.toString());


}

function handleLocalization(): Pose {

    // Read sensors
    let leftRPS: number = leftMotor.getRPS();
    let leftMPS: number;
    if (leftRPS != 0) {
        leftMPS = wheel_circumference / leftRPS;
    } else {
        leftMPS = 0.0;
    }
    let rightRPS: number = rightMotor.getRPS();
    let rightMPS: number;
    if (rightRPS != 0) {
        rightMPS = wheel_circumference / rightRPS;
    } else {
        rightMPS = 0.0;
    }

    let angle: Rotation = gyro.getRotation();

    // Calculate pose
    localizer.update(angle, leftMPS, rightMPS);

    // Return the pose
    return localizer.getPoseMeters();

}