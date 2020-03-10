
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
let wheel_diameter = 1; // TODO add real wheel diameter
let wheel_circumference = wheel_diameter * Math.PI;
let RampRate = 1;



/* Logging */

// TODO: Scrolling logs



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
        let deltaLeft: number = leftMeters - this.prevDistLeft;
        let deltaRight: number = rightMeters - this.prevDistRight;

        this.prevDistLeft = leftMeters;
        this.prevDistRight = rightMeters;

        let deltaPose: number = (deltaLeft + deltaRight) / 2.0;
        let angle: Rotation = currentAngle.rotateBy(this.gyroOffset);

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


/* ########## Main Robot Code ########## */

let gyro: Gyro = new Gyro();
let localizer: Localizer = new Localizer(gyro.getRotation(), new Pose(0, 0, createRotationDegrees(0.0)));

/**
 * All main code shall be run from here to reduce issues with cross-compiling to "blocks mode"
 */
function loop() {

    // Get the robot's current position
    let robotPose: Pose = handleLocalization();

}

function handleLocalization(): Pose {

    // Read sensors
    let leftMeters: number;
    let rightMeters: number;
    let angle: Rotation = gyro.getRotation();

    // Calculate pose
    localizer.update(angle, leftMeters, rightMeters);

    // Return the pose
    return localizer.getPoseMeters();

}