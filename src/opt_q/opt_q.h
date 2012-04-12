#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <fcntl.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#include <sys/stat.h>
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


float cmd_i[2] = {0, 0};
float return_int;
double cp;

void usage(char *prog_name, int exit_code) {
	printf("Usage: %s <consign> <simu stop>\n", prog_name);
	exit(exit_code);
}

void init(output *out_ptr, conf *c_ptr) {
	int k;
	FILE* fd_conf;
	char* conf_file;

	printf("[DEBUG] Initialising Simulation...\n");
	conf_file = (char *)ec_malloc(50);
	strcpy(conf_file, "./input.conf");
	if ((fd_conf = fopen(conf_file, "r")) == NULL)
		fatal("opening input config file");
	printf("[DEBUG] Loading vector input configuration [%p: \'%s\']\n", conf_file, conf_file);
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
	/*  Réduction des consignes et sorties ?  */
}

void get_pi(float consign, output *out_ptr, conf *c_ptr, float cp, float t) {
	// BF
	return_int = c_ptr->DELTA * (out_ptr->position[1] - consign) + return_int;
	//printf("[DEBUG] Computing integral return [%2.2f - %2.2f]\n", return_int, t);
	cmd_i[0] = (c_ptr->L2 * consign - (c_ptr->L2 * out_ptr->position[1] + c_ptr->L1 * out_ptr->speed[1] + c_ptr->L3 * return_int)) + c_ptr->WEIGTH * c_ptr->LENGTH * 9.8 * cp / CN;
	//printf("[DEBUG] Computing command [%2.2f - %2.2f]\n", cmd_i[0], t);

	if (c_ptr->SATURATION_FLAG) {
		//printf("[DEBUG] Check saturation [%2.2f - %2.2f]\n", c_ptr->SATURATION, t);
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

void mcc_model(output *out_ptr, conf *c_ptr, float t) {
	out_ptr->speed[0] = (cmd_i[1] * (1 - exp(-MU * c_ptr->DELTA / TM)) / MU) + (out_ptr->speed[1] * exp(-MU * c_ptr->DELTA / TM));
	//printf("[DEBUG] Computing WR [%2.2f - %2.2f]\n", out_ptr->speed[0], t);
	out_ptr->position[0] = (c_ptr->DELTA * out_ptr->speed[1]) + out_ptr->position[1];
	//printf("[DEBUG] Computing ThetaR [%2.2f - %2.2f]\n", out_ptr->position[0], t);
}

int plot_results() {
	FILE *gp;

	gp = popen(GNUPLOT_PATH, "w");
	printf("Figuring out gnuplot path...\n");
	fprintf(gp, "load \"config\"\n");
	printf("Loading gnuplot config file...\n");
	fflush(gp);
	getchar();
	pclose(gp);

	exit(EXIT_SUCCESS);
}
