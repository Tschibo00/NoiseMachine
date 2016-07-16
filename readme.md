# NoiseMachine
Arduino based hardware synth project

This Arduino based synth uses the same hardware design as the BadAssbasSSynth but with a different software
It's main purpose is to product all kinds of weird drone/noise sounds.
Best run through a large hall

3 oscillators combined with two functions.
First row: Press button 1-3 and turn pots to configure oscillator 1-3
	left: cutoff, middle: resonance, right: frequency
Second row: Activate/mute oscillator 1-3
Third row:  Select operator for oscillator 1+2: Add, Difference, XOR (threshold 200/255), XOR(threshold 40/255)
Fourth row: Select operator for oscillator 3+result of 1+2: Add, Difference, XOR (threshold 200/255), XOR(threshold 40/255)
Switch oscillator type of oscillator 1-3 by clicking left black button row 1-3 (selected waveform is displayed on green LEDs)
Press Black button top right +1-16 to load setting
Press black button top right and the left to it simultaneously +1-16 to write setting
