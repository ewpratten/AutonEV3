/**
 * All simulation components regarding the EV3 Brick
 */
var brick;
(function (brick) {
    /**
     * Simulation-only brick tools
     */
    var simulation;
    (function (simulation) {
        simulation.buttons = [false, false, false, false, false];
        /**
         * Register a simulation button press
         * @param btn Button ID
         */
        function registerButton(btn) {
            simulation.buttons[btn] = true;
        }
        simulation.registerButton = registerButton;
        /**
         * Read a simulation button
         * @param btn Button ID
         */
        function readButton(btn) {
            var val = simulation.buttons[btn];
            simulation.buttons[btn] = false;
            return val;
        }
        simulation.readButton = readButton;
    })(simulation = brick.simulation || (brick.simulation = {}));
    /**
     * Display canvas
     */
    var nxtCanvas = document.getElementById("NXT-screen").getContext("2d");
    nxtCanvas.font = "15px Arial";
    /**
     * Clear the NXT screen
     */
    function clearScreen() {
        nxtCanvas.fillStyle = "#97b5a6";
        nxtCanvas.fillRect(0, 0, 200, 200);
    }
    brick.clearScreen = clearScreen;
    /**
     * Show a message to the screen
     * @param message Message
     * @param line Line to write to
     */
    function showString(message, line) {
        nxtCanvas.strokeStyle = "#393939";
        nxtCanvas.fillStyle = "#393939";
        nxtCanvas.fillText(message, 10, line * 15);
    }
    brick.showString = showString;
    /* Buttons */
    var Button = /** @class */ (function () {
        function Button(port) {
            this.port = port;
        }
        Button.prototype.isPressed = function () {
            return simulation.readButton(this.port);
        };
        return Button;
    }());
    brick.Button = Button;
    brick.buttonUp = new Button(0);
    brick.buttonDown = new Button(1);
    brick.buttonLeft = new Button(2);
    brick.buttonRight = new Button(3);
    brick.buttonEnter = new Button(4);
})(brick || (brick = {}));
var sensors;
(function (sensors) {
    var GyroSensor = /** @class */ (function () {
        function GyroSensor(port) {
            this._angle = 0.0;
            this.port = port;
        }
        GyroSensor.prototype.angle = function () {
            return 0.0;
        };
        GyroSensor.prototype.updateSimulation = function (leftMPS, rightMPS, trackWidth) {
            // Calculate angle
            var omega = (leftMPS - rightMPS) / trackWidth;
            // Publish readings
            this._angle += omega;
        };
        return GyroSensor;
    }());
    sensors.GyroSensor = GyroSensor;
    sensors.gyro1 = new GyroSensor(1);
    sensors.gyro2 = new GyroSensor(2);
    sensors.gyro3 = new GyroSensor(3);
    sensors.gyro4 = new GyroSensor(4);
})(sensors || (sensors = {}));
var motors;
(function (motors) {
    var MotorBase = /** @class */ (function () {
        function MotorBase(port) {
            this.lastTime = 0.0;
            this.speed = 0.0;
            this._angle = 0.0;
            this.port = port;
        }
        MotorBase.prototype.setBrake = function (on) {
        };
        MotorBase.prototype.run = function (output) {
            this.speed = (output / 100) * ((this.invert) ? -1 : 1);
        };
        MotorBase.prototype.setInverted = function (on) {
            this.invert = on;
        };
        MotorBase.prototype.update = function () {
            var time = control.millis() * 10000;
            var dt = time - this.lastTime;
            this.lastTime = time;
            // Calc encoder position
            var rpm = (this.speed * ((this.invert) ? -1 : 1)) * 160;
            var revs = (rpm / 60.0) * dt; // RPM -> RPS -> Multiply by seconds to find rotations since last update
            this._angle += revs * 360;
        };
        MotorBase.prototype.angle = function () {
            return this._angle;
        };
        return MotorBase;
    }());
    motors.MotorBase = MotorBase;
    /* Motor defs */
    motors.largeA = new MotorBase("A");
    motors.largeB = new MotorBase("B");
    motors.largeC = new MotorBase("C");
    motors.largeD = new MotorBase("D");
})(motors || (motors = {}));
var control;
(function (control) {
    /**
     * Get the brick firmware version string
     */
    function deviceFirmwareVersion() {
        return "vSIM";
    }
    control.deviceFirmwareVersion = deviceFirmwareVersion;
    /**
     * Get fake serial number for the brick
     */
    function deviceSerialNumber() {
        return 5024;
    }
    control.deviceSerialNumber = deviceSerialNumber;
    /**
     * Get time in millis
     */
    function millis() {
        return new Date().getMilliseconds();
    }
    control.millis = millis;
})(control || (control = {}));
