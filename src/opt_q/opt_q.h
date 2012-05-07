#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <fcntl.h>
#include <time.h>
#ifdef MTRACE
#include <mcheck.h>
#endif	/*  MTRACE  */
#include "utils.h"

#define MU	1
#define TM	0.72
#define	WN	157
#define REF	6.28
#define CN	18

#define GNUPLOT_PATH "/usr/bin/gnuplot"

/*
 *Output vector
 */
typedef struct output output;
struct output {
	 float current;
	 float position[2];
	 float speed[2];
	 float tm;
	 float static_error;
	 float overshoot;
	 float compromise;
};

/*
 *Input vector
 */
typedef struct conf conf;
struct conf {
	float L1;
	float L2;
	float L3;
	float L1_PAS;
	float L2_PAS;
	float L3_PAS;
	int L1_MAX;
	int L2_MAX;
	int L3_MAX;
	float DELTA;
	int CP_FLAG;
	float LENGTH;
	float WEIGTH;
	float CONSIGN;
	int STOP;
	float SATURATION;
	int SATURATION_FLAG;
	float STATIC_TOLERANCE;
};

/*
 *Global variable for function calls convenience
 */
float cmd_i[2] = {0, 0};
float return_int;
double cp;

/*
 *Usually usage function
 */
void usage(char *prog_name, int exit_code) {
	printf("Usage: %s <consign> <simu stop>\n", prog_name);
	exit(exit_code);
}

/*
 *Initialise befor simulation
 */
void init(output *out_ptr, conf *c_ptr) {
	int k;
	FILE* fd_conf;
	char* conf_file;

	printf("[DEBUG] Initialising Simulation...\n");
	conf_file = (char *)ec_malloc(50);
	strcpy(conf_file, "./input.conf");
	if ((fd_conf = fopen(conf_file, "r")) == NULL)
		fatal("opening input config file");
	printf("[DEBUG] Loading vector input configuration [\'%s\': %p]\n", conf_file, conf_file);
	fscanf(fd_conf, "%f %f %f %f %f %f %d %d %d %f %d %f %f %f %d %f %d %f", &(c_ptr->L1), &(c_ptr->L2), &(c_ptr->L3), &(c_ptr->L1_PAS), &(c_ptr->L2_PAS), &(c_ptr->L3_PAS), &(c_ptr->L1_MAX), &(c_ptr->L2_MAX), &(c_ptr->L3_MAX), &(c_ptr->DELTA), &(c_ptr->CP_FLAG), &(c_ptr->LENGTH), &(c_ptr->WEIGTH), &(c_ptr->CONSIGN), &(c_ptr->STOP), &(c_ptr->SATURATION), &(c_ptr->SATURATION_FLAG), &(c_ptr->STATIC_TOLERANCE));

	c_ptr->CONSIGN = c_ptr->CONSIGN * REF / WN;
	for (k=0; k < 2; k++) {
		cmd_i[k] = 0;
		out_ptr->position[k] = 0;
		out_ptr->speed[k]=0;
	}
	out_ptr->compromise = 0;
	out_ptr->static_error = 0;
	out_ptr->tm = 0;
	out_ptr->overshoot = 0;
	out_ptr->current = 0;
	cp = 0;
	fclose(fd_conf);
	free(conf_file);
}

/*
 *Compute the command from feedback
 */
void get_pi(float consign, output *out_ptr, conf *c_ptr, float cp) {
	// BF
	return_int = c_ptr->DELTA * (out_ptr->position[1] - consign) + return_int;
	cmd_i[0] = (c_ptr->L2 * consign - (c_ptr->L2 * out_ptr->position[1] + c_ptr->L1 * out_ptr->speed[1] + c_ptr->L3 * return_int)) + c_ptr->WEIGTH * c_ptr->LENGTH * 9.8 * cp / CN;

	if (c_ptr->SATURATION_FLAG) {
		if (cmd_i[0] > c_ptr->SATURATION) {
			//printf("[DEBUG] Positive saturation [%2.2f - %2.2f]\n", c_ptr->SATURATION, t);
			cmd_i[0] = c_ptr->SATURATION;
		}
		else if (cmd_i[0] < -(c_ptr->SATURATION)) {
			//printf("[DEBUG] Negative saturation [%2.2f - %2.2f]\n", -c_ptr->SATURATION, t);
			cmd_i[0] = -(c_ptr->SATURATION);
		}
	}
}

/*
 *Compute output states variable
 */
void mcc_model(output *out_ptr, conf *c_ptr) {
	out_ptr->speed[0] = (cmd_i[1] * (1 - exp(-MU * c_ptr->DELTA / TM)) / MU) + (out_ptr->speed[1] * exp(-MU * c_ptr->DELTA / TM));
	out_ptr->position[0] = (c_ptr->DELTA * out_ptr->speed[1]) + out_ptr->position[1];
}

/*
 *Simulate a MCC
 */
int simu_mcc(conf* cpt_vect, output* cpt_out, int write, FILE* fd) {
	int cpt = 0;
	float t = 0;
	float max_pos = 0;
	cp = 0;

	for (cpt=0; cpt < 2; cpt++) {
		cmd_i[cpt] = 0;
		cpt_out->position[cpt] = 0;
		cpt_out->speed[cpt]=0;
	}
	cpt = 0;

	do {
		if (cpt_vect->CP_FLAG) 
			cp = sin(cpt_out->position[0] * REF);
		else
			cp = 0;

		get_pi(cpt_vect->CONSIGN, cpt_out, cpt_vect, cp);	 //compute cmd_i[0] of current loop
		mcc_model(cpt_out, cpt_vect);		 				//compute output[0]

		t += cpt_vect->DELTA;
	/*  increment indexes for next loop  */
		cpt_out->speed[1] = cpt_out->speed[0];
		cpt_out->position[1] = cpt_out->position[0];
		cmd_i[1] = cmd_i[0];
		
		if (fabs(cpt_out->position[0] - cpt_vect->CONSIGN) < (0.05*cpt_vect->CONSIGN)) {
			cpt++;
			if (cpt == 100) 
				cpt_out->tm = t-(cpt*cpt_vect->DELTA);
		}
				

		if (cpt_out->position[0] > max_pos)
			max_pos = cpt_out->position[0];

		if (write) {
			//cpt_out->position[0] = cpt_out->position[0] * WN / REF;
			cpt_out->speed[0] = cpt_out->speed[0];	// WN
			fprintf(fd, "%5.4f\t%5.4f\t%5.4f\t%2.2f\n", t, cpt_out->position[0]*WN/REF, cpt_out->speed[0], cp);
		}
		
	} while (t < cpt_vect->STOP); 
	cpt_out->static_error = fabs(cpt_out->position[0] - cpt_vect->CONSIGN)*100*WN/REF;
	cpt_out->overshoot = (max_pos - cpt_vect->CONSIGN)*100*WN/REF;

	return 0;
}

/*
 *Plot results with GNUPLOT
 */
int plot_results(char* config) {
	FILE *gp;

	gp = popen(GNUPLOT_PATH, "w");
	fprintf(gp, "load \"%s\"\n", config);
	//fprintf(gp, "load \"config_motor\"\n");
	printf("Loading gnuplot config file [config_motor]...\n");
	fflush(gp);
	getchar();
	pclose(gp);

	exit(EXIT_SUCCESS);
}
