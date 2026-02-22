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

### Converting samples
`process_log.m` creates an interpolated sample set from the logfile with 1 ms sample time.

### System identification
The input and output vectors can be imported in the `systemIdentification` tool to create a model of the system. Alternatively, you can open the saved session, which already has a pretty well fitting nonlinear Hammerstein-Wiener model. The best results so far were achieved by using piecewise nonlinearity with 34 units on the input, and 68 units on the output.

### PID tuning
The Simulink model has a simple step-response test setup. Before opening it, you should open the `systemIdentification` tool, open the saved session, and copy the model to the workspace.