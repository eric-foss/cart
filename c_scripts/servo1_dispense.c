#include <stdio.h>
#include <getopt.h>
#include <stdlib.h> // for atoi
#include <signal.h>
#include <rc/time.h>
#include <rc/adc.h>
#include <rc/dsm.h>
#include <rc/servo.h>

static int running;

// interrupt handler to catch ctrl-c
static void __signal_handler(__attribute__ ((unused)) int dummy)
{
        running=0;
        return;
}

int main(int argc, char *argv[])
{

	int ch = -1;
	double servo_pos = 0.0;
	int frequency_hz = 50;
	int counter = 0;
	int c;

	opterr = 0;
	while ((c = getopt(argc, argv, "c:p:")) != -1){
		switch (c){
			case 'c':
				ch = atoi(optarg);
				if(ch < 1 || ch > 8){
					fprintf(stderr, "ERROR channel option must be between 1 and 8 \n");
					return -1;
				}
				break;
			case 'p':
				servo_pos = atof(optarg);
				if(servo_pos > 1.5 || servo_pos < -1.5){
					fprintf(stderr, "Servo position must be between -1.5 and 1.5 \n");
					return -1;
				}
				break;
			default:
				printf("\nInvalid Argument \n");
				return -1;
		}
	}

	if (ch == -1 || servo_pos == 0.0) {
		fprintf(stderr, "ERROR: Both channel (-c) and position (-p) must be specified \n");
		return -1;
	}

	signal(SIGINT, __signal_handler);
	running = 1;

	// read adc to make sure battery is connected
        if(rc_adc_init()){
                fprintf(stderr,"ERROR: failed to run rc_adc_init()\n");
                return -1;
        }
        if(rc_adc_batt()<6.0){
                fprintf(stderr,"ERROR: battery disconnected or insufficiently charged to drive servos\n");
                return -1;
        }
        rc_adc_cleanup();

	// initialize PRU
        if(rc_servo_init()) return -1;

	// turn on power
        printf("Turning On 6V Servo Power Rail\n");
        rc_servo_power_rail_en(1);



	while(running){

		rc_servo_send_pulse_normalized(ch, servo_pos);

		counter = counter + 1;

		rc_usleep(100000/frequency_hz);

		if (counter > 500) {
			running = 0;
		}
	}



	rc_usleep(50000);
	rc_servo_power_rail_en(0);
	rc_servo_cleanup();

}
