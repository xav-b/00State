/* 
 * Simulation de la commande PI
 * sur le modèle d'une MCC
 * Groupe M2AA007
 *	
 * TODOLIST: 
 * calculer le dépassement, la précision et les temps associés, analyse des résultats
 * intégrer les calculs par réduction quadratique des correcteurs
 * employer une méthode d'auto calcul plutôt que par saisie ? identification de la charge par comparaison avec l'attendu linéaire?
 * intégrer la mesure du courant (et sa limitation ?)
 * utiliser les graphiques 3D pour de meilleurs analyses 
 * intégrer d'autres types de charge
 * vérifier la validité du modèle
 * Faire de plus beaux graphiques (R ?)
 * Données du moteur dans fichier à part, ranger tout ça
 * graph interactif et animé
 * intégration du choix de la méthode d'asservissement
 * intégration des erreurs réelles (mesure, etc... cf code micropro)
 * optimisation
 * interface graphique
 * version matlab
 * 
 */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <fcntl.h>
#include <sys/stat.h>
#ifdef MTRACE
#include <mcheck.h>
#endif	/*  MTRACE  */

#define	DELTA	0.01
/*
 *#define	L1	20.60
 *#define	L2	76.64
 *#define	L3	3.16
 */

#define MU	1
#define TM	0.72
#define	WN	157
#define REF	6.28
#define CN	18
#define L	0.5
#define M	0.5

#define GNUPLOT_PATH "/usr/bin/gnuplot"

typedef struct output output;
struct output {
	 float current;
	 float position[2];
	 float speed[2];
};

typedef struct correct correct;
struct correct {
	 int L1;
	 int L2;
	 int L3;
};

float cmd_i[2] = {0, 0};
float return_int;
double cp;

void usage(char *prog_name, int exit_code) {
	printf("Usage: %s <consign> <simu stop>\n", prog_name);
	exit(exit_code);
}

void init(output *out_ptr) {
	int k;

	for (k=0; k < 2; k++) {
		cmd_i[k] = 0;
		out_ptr->position[k] = 0;
		out_ptr->speed[k]=0;
	}
	out_ptr->current = 0;
	cp = 0;
	/*  Réduction des consignes et sorties ?  */
}

void get_pi(float consign, output *out_ptr, correct *gain_ptr) {
	// BF
	return_int = DELTA * (out_ptr->position[1] - consign) + return_int;
	cmd_i[0] = (gain_ptr->L2 * consign - (gain_ptr->L2 * out_ptr->position[1] + gain_ptr->L1 * out_ptr->speed[1] + gain_ptr->L3 * return_int)) + M * L * 9.8 * cp / CN;
}

void mcc_model(output *out_ptr) {
	out_ptr->speed[0] = (cmd_i[1] * (1 - exp(-MU * DELTA / TM)) / MU) + (out_ptr->speed[1] * exp(-MU * DELTA / TM));
	out_ptr->position[0] = (DELTA * out_ptr->speed[1]) + out_ptr->position[1];
}

int plot_results() {
	FILE *gp;

	gp = popen(GNUPLOT_PATH, "w");
	fprintf(gp, "load \"config\"\n");
	fflush(gp);
	getchar();
	pclose(gp);

	exit(EXIT_SUCCESS);
}


int main(int argc, char *argv[]) {
	//mtrace();
	//export MALLOC_TRACE=memory.log
	float consign=0, max_pos=0, t=0, settled_time = 0;
	int stop=0;
	int cpt = 0;
	int i = 0;
	char *datafile;
	FILE* fd;
	output val_out;
	correct gain; 

	printf("\n-----   Numeric simulation for MCC command   ------\n");
	if (argc < 3)
		usage(argv[0], 1);
	
	/*
	 *init(&val_out, &gain);		//Global variables initialisation
	 */
	gain.L1 = 0;
	gain.L2 = 0;
	gain.L3 = 0;
	consign=atoi(argv[1]) * 6.3 / 157; 
	stop = atoi(argv[2]);
	
	datafile = (char *)malloc(20);
	strcpy(datafile, "./results");
	printf("[DEBUG] datafile @  %p: \'%s\'\n", datafile, datafile);
	fd = fopen(datafile, "w");
	
	
	while (gain.L1 < 30) {
	    gain.L1 += 1;
	    while (gain.L2 < 80) {
	        gain.L2+=1;
	        while (gain.L3 < 4) {
	            gain.L3+=1;
				do {
					cp = sin(val_out.position[0] * REF);

					get_pi(consign, &val_out, &gain);	 //compute cmd_i[0] of current loop
					mcc_model(&val_out);		 //compute output[0]

					t += DELTA;
				/*  increment indexes for next loop  */
					val_out.speed[1] = val_out.speed[0];
					val_out.position[1] = val_out.position[0];
					cmd_i[1] = cmd_i[0];
					
					val_out.position[0] = val_out.position[0] * WN / REF;
					val_out.speed[0] = val_out.speed[0];	//* WN
					/*
					 *fprintf(fd, "%5.4f\t%5.4f\t%5.4f\t%2.2f\n", t, val_out.position[0], val_out.speed[0], cp);
					 */
					
					if (fabs(val_out.position[0] - atoi(argv[1])) < (0.05*atoi(argv[1])))
						   cpt++;
					if (cpt == 50) {
						settled_time = t-(cpt*DELTA);
						/*printf("Settled in %2.2f\n", settled_time);*/
						//break;
						/*
						 *cpt = 0;
						 */
					}		
					if (val_out.position[0] > max_pos)
						max_pos = val_out.position[0];

				} while (t < stop); 
				i++;
				fprintf(fd, "%d\t%4.2f\t%2.2f\t%2.2f\t%d\t%d\t%d\n", i, fabs(max_pos - atoi(argv[1]))*100, settled_time, fabs(val_out.position[0] - atoi(argv[1]))*100, gain.L1, gain.L2, gain.L3);
				/*fprintf(fd, "%d\t%4.2f\t%2.2f\t%2.2f\t%d\t%d\t%d\n", i, val_out.position[0], settled_time, fabs(val_out.position[0] - atoi(argv[1]))*100, gain.L1, gain.L2, gain.L3);*/
				/*printf("L1: %d\tL2: %d\tL3: %d\n", gain.L1, gain.L2, gain.L3);*/
				settled_time = 0;
				max_pos = 0;
				t = 0;
				cpt = 0;
				init(&val_out);		//Global variables initialisation
	
	         }
	         gain.L3 = 0;
	     }
	     gain.L2 = 0;
	 }
	 
	
	 /*
	  *printf("\n[DEBUG] loop for ended, closing file\n\n");
	  *printf("position max atteinte: %2.2f\n", max_pos);
	  *printf("Which is a %2.2f overshoot\n", fabs((max_pos - atoi(argv[1])))*100);
	  *printf("erreur statique: %2.2f\n", fabs(val_out.position[0] - atoi(argv[1]))*100);
	  */
	 
	fclose(fd);
	free(datafile);
	plot_results();
	
	return 0;
}
