
/**
 * All simulation components regarding the EV3 Brick
 */
namespace brick {

    /**
     * Simulation-only brick tools
     */
    export namespace simulation {

        export let buttons = [false, false, false, false, false];

        /**
         * Register a simulation button press
         * @param btn Button ID
         */
        export function registerButton(btn: number) {
            buttons[btn] = true;
        }

        /**
         * Read a simulation button
         * @param btn Button ID
         */
        export function readButton(btn: number): boolean {
            let val = simulation.buttons[btn];
            simulation.buttons[btn] = false;
            return val;
        }

    }

    /**
     * Display canvas
     */
    let nxtCanvas: CanvasRenderingContext2D = document.getElementById("NXT-screen").getContext("2d");
    nxtCanvas.font = "15px Arial";

    /**
     * Clear the NXT screen
     */
    export function clearScreen() {
        nxtCanvas.fillStyle = "#97b5a6";
        nxtCanvas.fillRect(0, 0, 200, 200);
    }

    /**
     * Show a message to the screen
     * @param message Message
     * @param line Line to write to
     */
    export function showString(message: string, line: number) {
        nxtCanvas.strokeStyle = "#393939";
        nxtCanvas.fillStyle = "#393939";
        nxtCanvas.fillText(message, 10, line * 15);
    }

    /* Buttons */
    export class Button {

        private port: number;

        constructor(port: number) {
            this.port = port;
        }

        public isPressed() {
            return simulation.readButton(this.port);
        }
    }

    export let buttonUp: Button = new Button(0);
    export let buttonDown: Button = new Button(1);
    export let buttonLeft: Button = new Button(2);
    export let buttonRight: Button = new Button(3);
    export let buttonEnter: Button = new Button(4);

}

namespace sensors {
    export class GyroSensor {

        private port: number;
        private _angle: number = 0.0;

        constructor(port: number) {
            this.port = port;
        }

        public angle(): number {
            return 0.0;
        }

        public updateSimulation(leftMPS: number, rightMPS: number, trackWidth: number) {
            // Calculate angle
            let omega: number = (leftMPS - rightMPS) / trackWidth;

            // Publish readings
            this._angle += omega;
        }
    }

    export let gyro1: GyroSensor = new GyroSensor(1);
    export let gyro2: GyroSensor = new GyroSensor(2);
    export let gyro3: GyroSensor = new GyroSensor(3);
    export let gyro4: GyroSensor = new GyroSensor(4);
}

namespace motors {

    export class MotorBase {

        private port: string;
        private lastTime: number = 0.0;
        private invert: boolean;
        private speed: number = 0.0;
        private _angle: number = 0.0;

        constructor(port: string) {
            this.port = port;
        }

        public setBrake(on: boolean) {
        }

        public run(output: number) {
            this.speed = (output / 100) * ((this.invert) ? -1 : 1);

        }

        public setInverted(on: boolean) {
            this.invert = on;
        }

        public update() {
            let time = control.millis() * 10_000;
            let dt = time - this.lastTime;
            this.lastTime = time;

            // Calc encoder position
            let rpm: number = (this.speed * ((this.invert) ? -1 : 1)) * 160;
            let revs: number = (rpm / 60.0) * dt; // RPM -> RPS -> Multiply by seconds to find rotations since last update
            this._angle += revs * 360;

        }

        public angle(): number {
            return this._angle;
        }
    }

    /* Motor defs */
    export let largeA: MotorBase = new MotorBase("A");
    export let largeB: MotorBase = new MotorBase("B");
    export let largeC: MotorBase = new MotorBase("C");
    export let largeD: MotorBase = new MotorBase("D");



}

namespace control {

    /**
     * Get the brick firmware version string
     */
    export function deviceFirmwareVersion(): string {
        return "vSIM";
    }

    /**
     * Get fake serial number for the brick
     */
    export function deviceSerialNumber(): number {
        return 5024;
    }

    /**
     * Get time in millis
     */
    export function millis(): number {
        return new Date().getMilliseconds();
    }
}

