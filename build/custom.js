// Constants
var left_motor_port = "a";
var right_motor_port = "b";
var wheel_diameter = 0.055; // TODO add real wheel diameter
var gyroscope = sensors.gyro1;
var wheel_circumference = wheel_diameter * Math.PI;
var RampRate = 0.0;
var trackWidth = 0.1175;
/* Logging */
var max_log_lines = 12;
var active_log_messages = ["", ""];
function log(message) {
    active_log_messages.unshift(message);
    if (active_log_messages.length > max_log_lines) {
        active_log_messages.pop();
    }
    // Clear the screen to make way
    brick.clearScreen();
    for (var i = 0; i < active_log_messages.length - 1; i++) {
        // Disp the message to the screen
        brick.showString(active_log_messages[i], max_log_lines - i);
    }
}
/* ########## Utils ########## */
/**
 * Convert degrees to radians
 * @param degrees Degrees
 */
function d2r(degrees) {
    var pi = Math.PI;
    return degrees * (pi / 180);
}
/**
 * Convert radians to degrees
 * @param radians Radians
 */
function r2d(radians) {
    var pi = Math.PI;
    return radians * (180 / pi);
}
function hypot(x, y) {
    var max = 0;
    var s = 0;
    var arg = Math.abs(y);
    if (arg > max) {
        s *= (max / arg) * (max / arg);
        max = arg;
    }
    s += arg === 0 && max === 0 ? 0 : (arg / max) * (arg / max);
    return max === 1 / 0 ? 1 / 0 : max * Math.sqrt(s);
}
function getWrappedError(currentAngle, desiredAngle) {
    var phi = Math.abs(currentAngle - desiredAngle) % 360; // This is either the distance or 360 - distance
    var distance = phi > 180 ? 360 - phi : phi;
    // Determine the sign (is the difference positive of negative)
    var sign = (currentAngle - desiredAngle >= 0 && currentAngle - desiredAngle <= 180)
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
function clamp(val, min, max) {
    return Math.max(min, Math.min(val, max));
}
/**
 * Format a float to 2 decimals
 * @param val Value to format
 */
function pf2(val) {
    return Math.round(val * 100) / 100;
}
function isSimulation() {
    return control.deviceFirmwareVersion() == "vSIM";
}
function epsilonEquals(a, b, epsilon) {
    return (a - epsilon <= b) && (a + epsilon >= b);
}
/* ########## Gyro Class ########## */
var Gyro = /** @class */ (function () {
    function Gyro() {
    }
    Gyro.prototype.getDegrees = function () {
        return gyroscope.angle();
    };
    Gyro.prototype.getRotation = function () {
        return new Rotation(d2r(this.getDegrees()));
    };
    return Gyro;
}());
/* ########## Kinematics ########## */
var DriveSignal = /** @class */ (function () {
    function DriveSignal() {
    }
    /**
     *
     * @param left left goal value
     * @param right right goal value
     */
    DriveSignal.prototype.DriveSignal = function (left, right) {
        this.left_goal = left;
        this.right_goal = right;
    };
    return DriveSignal;
}());
/* ########## Localization ########## */
var Rotation = /** @class */ (function () {
    /**
     *
     * @param rads Angle in radians
     */
    function Rotation(rads) {
        this.rads = rads;
    }
    Rotation.prototype.getDegrees = function () {
        return r2d(this.rads);
    };
    Rotation.prototype.getRadians = function () {
        return this.rads;
    };
    Rotation.prototype.setRadians = function (rads) {
        this.rads = rads;
    };
    Rotation.prototype.fromPoint = function (x, y) {
        var mag = hypot(x, y);
        var sin;
        var cos;
        if (mag > 1e-6) {
            sin = y / mag;
            cos = x / mag;
        }
        else {
            sin = 0;
            cos = 1;
        }
        return new Rotation(Math.atan2(sin, cos));
    };
    Rotation.prototype.rotateBy = function (other) {
        var x = Math.cos(this.rads) * Math.cos(other.getRadians()) - Math.sin(this.rads) * Math.sin(other.getRadians());
        var y = Math.cos(this.rads) * Math.sin(other.getRadians()) - Math.sin(this.rads) * Math.cos(other.getRadians());
        return this.fromPoint(x, y);
    };
    Rotation.prototype.unaryMinus = function () {
        return new Rotation(this.rads * -1);
    };
    Rotation.prototype.minus = function (other) {
        return this.rotateBy(other.unaryMinus());
    };
    return Rotation;
}());
/**
 * Create a rotation using degrees
 * @param degrees Degree angle
 */
function createRotationDegrees(degrees) {
    return new Rotation(r2d(degrees));
}
/**
 * A position in 2D space
 */
var Pose = /** @class */ (function () {
    /**
     * Create a pose in 2D space
     * @param x X component
     * @param y Y component
     * @param theta Angle
     */
    function Pose(x, y, theta) {
        if (theta === void 0) { theta = new Rotation(0.0); }
        this.x = x;
        this.y = y;
        this.theta = theta;
    }
    Pose.prototype.toString = function () {
        return "Pose<" + pf2(this.x) + ", " + pf2(this.y) + ", " + pf2(this.theta.getDegrees()) + ">";
    };
    return Pose;
}());
/**
 * Create a pose that follows a scaling factor
 * @param x  X component
 * @param y  Y component
 * @param theta  Angle in degrees
 * @param scale the scaling multiplier
 * @return returns a scaled posed
 */
function createScaledPose(x, y, theta, scale) {
    return new Pose(x * scale, y * scale, createRotationDegrees(theta));
}
/**
 * A tool for localizing a differential drivebase
 */
var Localizer = /** @class */ (function () {
    function Localizer(startAngle, startPose) {
        this.pose = startPose;
        this.gyroOffset = startAngle.minus(startPose.theta);
        this.prevAngle = startPose.theta;
        this.prevDistLeft = 0.0;
        this.prevDistRight = 0.0;
    }
    Localizer.prototype.reset = function (pose, angle) {
        this.pose = pose;
        this.prevAngle = pose.theta;
        this.gyroOffset = pose.theta.minus(angle);
        this.prevDistLeft = 0.0;
        this.prevDistRight = 0.0;
    };
    Localizer.prototype.update = function (currentAngle, leftMeters, rightMeters) {
        var deltaLeft = leftMeters;
        var deltaRight = rightMeters;
        this.prevDistLeft = leftMeters;
        this.prevDistRight = rightMeters;
        var deltaPose = (deltaLeft + deltaRight) / 2.0;
        var angle = currentAngle.rotateBy(new Rotation(getWrappedError(this.gyroOffset.getRadians(), 0)));
        // TODO: These may need to be flipped cos/sin
        var newX = this.pose.x + (deltaPose * Math.sin(angle.getRadians()));
        var newY = this.pose.y + (deltaPose * Math.cos(angle.getRadians()));
        this.pose.x = newX;
        this.pose.y = newY;
        this.pose.theta = angle;
    };
    Localizer.prototype.getPoseMeters = function () {
        return this.pose;
    };
    return Localizer;
}());
/* ########## Drive code ########## */
var Motor = /** @class */ (function () {
    function Motor(motor) {
        this.motor = motor;
    }
    Motor.prototype.set = function (output) {
        this.motor.run(output * 100);
        this.output = output;
    };
    Motor.prototype.get = function () {
        return this.output;
    };
    Motor.prototype.setBrakes = function (on) {
        this.motor.setBrake(on);
    };
    Motor.prototype.setInverted = function (invert) {
        this.motor.setInverted(invert);
        this.inverted = invert;
    };
    Motor.prototype.update = function () {
        var angle = this.motor.angle();
        var dtheta = getWrappedError(angle, this.lastAngle);
        this.lastAngle = angle;
        var time = control.millis() / 1000;
        var dt = time - this.lastTime;
        this.lastTime = time;
        this.rps = dtheta * dt;
    };
    Motor.prototype.getRPS = function () {
        return this.rps * ((this.inverted) ? -1 : 1);
    };
    Motor.prototype.getRPM = function () {
        return (this.rps / 60) * ((this.inverted) ? -1 : 1);
    };
    return Motor;
}());
/* ####### Grouping Of Motors ####### */
var MotorGrouping = /** @class */ (function () {
    function MotorGrouping(Motor1, Motor2) {
        this.motors.push(Motor1);
        this.motors.push(Motor2);
        this.motorOneOutput = Motor1.get();
        this.motorTwoOutput = Motor2.get();
    }
    /**
     * Sets output for both numbers
     *
     * @param output motor power between -1,1
     */
    MotorGrouping.prototype.setOutput = function (output) {
        this.motors[0].set(output * 100);
        this.motorOneOutput = output;
        this.motors[1].set(output * 100);
        this.motorTwoOutput = output;
        log("Setting Group Motor Speeds To: " + output);
    };
    /**
     * Sets both motor outputs individually
     *
     * @param motorOneOutput Motor 1's output between -1,1
     * @param motorTwoOutput Motor 2's output between -1,1
     */
    MotorGrouping.prototype.setOutputPerMotor = function (motorOneOutput, motorTwoOutput) {
        this.motors[0].set(motorOneOutput * 100);
        this.motorOneOutput = motorOneOutput;
        this.motors[1].set(motorTwoOutput * 100);
        this.motorTwoOutput = motorTwoOutput;
        log("Setting Motor 1 Speed to: " + motorOneOutput);
        log("Setting Motor 2 Speed to: " + motorTwoOutput);
    };
    /**
     * Sets the brakes state
     *
     * @param state the state the breaks should be in
     */
    MotorGrouping.prototype.setBrakes = function (state) {
        for (var _i = 0, _a = this.motors; _i < _a.length; _i++) {
            var motor = _a[_i];
            motor.setBrakes(state);
        }
    };
    /**
     * Updates both motors in the group
     */
    MotorGrouping.prototype.updateMotors = function () {
        for (var _i = 0, _a = this.motors; _i < _a.length; _i++) {
            var motor = _a[_i];
            motor.update();
        }
    };
    /**
     * gets a motor from the motor list
     * @param motorNumber the motor number in the array motor 1 = 0, motor 2 = 1
     * @returns returns a motor
     */
    MotorGrouping.prototype.getMotor = function (motorNumber) {
        return this.motors[motorNumber];
    };
    return MotorGrouping;
}());
// Motor defs
var leftMotor = new Motor(motors.largeA);
var rightMotor = new Motor(motors.largeB);
function arcadeDrive(speed, turn) {
    var left = speed + turn;
    var right = speed - turn;
    var magnitude = Math.max(left, right);
    if (magnitude > 1.0) {
        left /= magnitude;
        right /= magnitude;
    }
    // Send tank command
    leftMotor.set(left);
    rightMotor.set(right);
}
function initLib() {
    log("Loaded Lib5K-FLL v1");
    log("Talking with brick");
    log("Brick ID: " + control.deviceSerialNumber());
    log("Brick FW: " + control.deviceFirmwareVersion());
    log("Is simulation: " + isSimulation());
    log("*Robot code starting*");
    init();
}
/* ########## Main Robot Code ########## */
var gyro = new Gyro();
var localizer = new Localizer(gyro.getRotation(), new Pose(0, 0, createRotationDegrees(0.0)));
// Path definition
var scale = 1.0;
var waypoints = [new Pose(0.0, 0.0, createRotationDegrees(0.0)), new Pose(1.0, 0.0, createRotationDegrees(0.0))];
function init() {
    log("Setting motor modes");
    leftMotor.setBrakes(true);
    rightMotor.setBrakes(true);
    log("Setting motor inversions");
    leftMotor.setInverted(true);
    rightMotor.setInverted(true);
    log("Awaiting button press");
}
var canRunCode = false;
var simulation = isSimulation();
/**
 * All main code shall be run from here to reduce issues with cross-compiling to "blocks mode"
 */
function loop() {
    // Update both motors
    leftMotor.update();
    rightMotor.update();
    // Wait for button press to run code
    if (!canRunCode) {
        if (brick.buttonDown.isPressed()) {
            log("Running program");
            canRunCode = true;
        }
        return;
    }
    else {
        if (brick.buttonUp.isPressed()) {
            arcadeDrive(0, 0);
            log("Program Stopped");
            canRunCode = false;
            return;
        }
    }
    // Get the robot's current position
    var robotPose = handleLocalization();
    log("" + robotPose.toString());
    // Move to the goal
    driveToPoint(robotPose, new Pose(0.5, 0.0, new Rotation(0.0)));
}
function handleLocalization() {
    // Read sensors
    var leftRPS = leftMotor.getRPS();
    var leftMPS;
    if (leftRPS != 0) {
        leftMPS = wheel_circumference / leftRPS;
    }
    else {
        leftMPS = 0.0;
    }
    var rightRPS = rightMotor.getRPS();
    var rightMPS;
    if (rightRPS != 0) {
        rightMPS = wheel_circumference / rightRPS;
    }
    else {
        rightMPS = 0.0;
    }
    // Update the gyro in simulation
    if (simulation) {
        // Update the motor simulations
        motors.largeA.update();
        motors.largeB.update();
        // Update the gyro simulation
        sensors.gyro1.updateSimulation(leftMPS, rightMPS, trackWidth);
    }
    var angle = gyro.getRotation();
    // Calculate pose
    localizer.update(angle, leftMPS, rightMPS);
    // Return the pose
    return localizer.getPoseMeters();
}
var kp = 0.3;
var eps = 0.1;
function driveToPoint(current, goal) {
    var alpha = Math.atan2(goal.x - current.x, goal.y - current.y) - current.theta.getRadians();
    var delta = Math.atan2(2.0 * trackWidth * Math.sin(alpha), 1.0) * kp;
    var speed = clamp(alpha, -1, 1);
    speed = 0.0;
    if (epsilonEquals(goal.x - current.x, 0, eps) && epsilonEquals(goal.y - current.y, 0, eps)) {
        speed = 0.0;
    }
    // Simple drive
    arcadeDrive(speed, delta);
}
function drivePath(motor1, Motor2) {
    motor1.set(.5);
    motor1.set(.5);
}
