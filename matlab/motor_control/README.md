# Tuning PID controller for motor speed control

## File list

| File               | Description |
|--------------------|-------------|
| log.csv            | Measured step responses of the motor. Every entry has a timestamp taken from a 40 kHz timer, the speed setting, and the encoder change since the last sample.
| importfile.m       | Generated CSV importer  |
| process_log.m      | Import the CSV, convert timestamps, calculate motor speeds |
| sysident.sid       | systemIdentification tool session  |
| pidcontroller.slx  | Simulink model with PID controller  |

## Tutorial
### Shaft to wheel ratio
The ratio between the wheel and the shaft (thus encoder) is needed if we want to control the robot with physical values (eg.: speed in $\frac{m}{s}$). It has been measured with the following snippet:
```cpp
int application() {
    uint32_t left_init, right_init;

    left_init = getEncoderPosition(MOT_L);
    right_init = getEncoderPosition(MOT_R);

    while(1){
    	lcdPrintf(0,0,"Left enc: %d", getEncoderPosition(MOT_L) - left_init);
    	lcdPrintf(1,0,"Right enc: %d", getEncoderPosition(MOT_R) - right_init);
    	delayMs(100);
    }
}
```
The wheel has been marked to know where a rotation is complete, then rotated around 20 times to reduce the error due to misalignment. The software counted 19410 encoder ticks which gives $\frac{19410}{20}=970.5 \frac{ticks}{rot}$.

### Converting samples
`process_log.m` creates an interpolated sample set from the logfile with 1 ms sample time.


### System identification
The input and output vectors can be imported in the `systemIdentification` tool to create a model of the system. Alternatively, you can open the saved session, which already has a pretty well fitting nonlinear Hammerstein-Wiener model. The best results so far were achieved by using piecewise nonlinearity with 34 units on the input, and 68 units on the output.

### PID tuning
The PID tuning has been done with the MATLAB PID tuner application to achieve the desired step response. The identified plant and the designed controller can be found in `plant_and_controller_model.mat`. A Simulink model will be prepared to test the model in simulation.