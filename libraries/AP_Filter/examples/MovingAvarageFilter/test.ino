#include <MovingAvarageFilter.h>

MovingAvarageFilter movingAvarageFilter(20);

void setup() {
	Serial.begin(115200);
}

void loop() {

	// declare input and output variables
	float input = 1; // without a real input, looking at the step respons (input at unity, 1)
	float output = 0;

	for (int n = 0; n < 100 + 2; n++) {
		Serial.print("n= ");		// print the sample number
		Serial.println(n, DEC);
		Serial.println("Now calling fir...");
		output = movingAvarageFilter.process(input);		// here we call the fir routine with the input. The value 'fir' spits out is stored in the output variable.
		Serial.print("fir presented the following value= ");
		Serial.println(output);		// just for debugging or to understand what it does, print the output value
	}

	while (true) {};			// endless loop
}