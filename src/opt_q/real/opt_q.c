#include <unistd.h>
#include <sys/wait.h>

#include "opt_q.h"

int main(int argc, char **argv) {
	//mtrace();
	//export MALLOC_TRACE=memory.log
	float L2_save = 0, L3_save = 0;
	float tot_simu = 0;
	int i = 0;
	float best_all[7] = {0,0,0,101,101,101,101};
	float best_tm[6] = {0,0,0,101,101,101};
	float best_overshoot[6] = {0,0,0,101,101,101};
	char *datafile;
	FILE* fd;
	int write = 0;
	output val_out;
	conf vect;

	printf("\n-----   Numeric simulation for MCC command   ------\n");
    if (argc < 2) 
        usage( "./opt_q", -2 );

	init(&val_out, &vect);		//Global variables initialisation

	L2_save = vect.L2;
	L3_save = vect.L3;
	tot_simu = ((vect.L1_MAX - vect.L1)/vect.L1_PAS)*((vect.L2_MAX - vect.L2)/vect.L2_PAS)*((vect.L3_MAX - vect.L3)/vect.L3_PAS);
	
	datafile = (char *)malloc(20);
	strcpy(datafile, "./results");
	printf("[DEBUG] Loading datafile [\'%s\': %p]\n", datafile, datafile);
	fd = fopen(datafile, "w");

	printf("[DEBUG] Mode: %s\n", argv[1]);
	printf("[DEBUG] %.1f programmed simulations... [hint <CTRL-C> to quit]\n", tot_simu);
	fprintf(stderr, "	Progress [ ");

	while (vect.L1 < vect.L1_MAX) {
	    vect.L1 += vect.L1_PAS;
		while (vect.L2 < vect.L2_MAX) {
			vect.L2+=vect.L2_PAS;
			while (vect.L3 < vect.L3_MAX) {
				vect.L3+=vect.L3_PAS;
				
				simu_mcc(&vect, &val_out, write, fd);
				i++;

				if ( strcmp(argv[1], "test") == 0 ) 
					fprintf(fd, "%d\t%4.2f\t%2.2f\t%2.2f\t%2.2f\t%2.2f\t%2.2f\n", i, val_out.overshoot, val_out.tm, val_out.static_error, vect.L1, vect.L2, vect.L3);

				if (val_out.static_error < vect.STATIC_TOLERANCE) {
					val_out.compromise = val_out.static_error + val_out.tm + (val_out.overshoot/20);
					if (val_out.compromise < best_all[3])  {
						best_all[0] = vect.L1; 
						best_all[1] = vect.L2; 
						best_all[2] = vect.L3; 
						best_all[3] = val_out.compromise;
						best_all[4] = val_out.tm; 
						best_all[5] = val_out.static_error; 
						best_all[6] = val_out.overshoot; 
					}
					if (val_out.tm < best_tm[3])  {
						best_tm[0] = vect.L1; 
						best_tm[1] = vect.L2; 
						best_tm[2] = vect.L3; 
						best_tm[3] = val_out.tm;
						best_tm[4] = val_out.static_error; 
						best_tm[5] = val_out.overshoot; 
					}
					if (val_out.overshoot < best_overshoot[3])  {
						best_overshoot[0] = vect.L1; 
						best_overshoot[1] = vect.L2; 
						best_overshoot[2] = vect.L3; 
						best_overshoot[3] = val_out.overshoot;
						best_overshoot[4] = val_out.tm;
						best_overshoot[5] = val_out.static_error;
					}
				}
				/*else
					printf("[!][%d] Wrong configuration: %f [%f %f %f]\n", i, val_out.static_error, vect.L1, vect.L2, vect.L3);*/

             }
             vect.L3 = L3_save;
		 }
		 vect.L2 = L2_save;
		 fprintf(stderr, "=");
	 }
 	
	fprintf(stderr, "> ]\nComputation ended, printing results...\n\n");
	printf("Best compromise	%.2f	[%.2f %.2f %.2f]\n", best_all[3], best_all[0], best_all[1], best_all[2]);
	printf("	- Establishment	%.2fs\n", best_all[4]);
	printf("	- Static error	%.2f%\n", best_all[5]);
	printf("	- Overshoot	%.2f%\n", best_all[6]);
	printf("Fastest		%.2fs	[%.2f %.2f %.2f]\n", best_tm[3], best_tm[0], best_tm[1], best_tm[2]);
	printf("	- Static error	%.2f%\n", best_tm[4]);
	printf("	- Overshoot	%.2f%\n", best_tm[5]);
	printf("Most stable	%.2f% 	[%.2f %.2f %.2f]\n", best_overshoot[3], best_overshoot[0], best_overshoot[1], best_overshoot[2]);
	printf("	- Establishment	%.2fs\n", best_overshoot[4]);
	printf("	- Static Error	%.2f%\n", best_overshoot[5]);
	
	write = 1;
	vect.L1 = best_all[0]; vect.L2 = best_all[1]; vect.L3 = best_all[2];
	simu_mcc(&vect, &val_out, write, fd);
	fprintf(fd, "\n");
	vect.L1 = best_tm[0]; vect.L2 = best_tm[1]; vect.L3 = best_tm[2];
	simu_mcc(&vect, &val_out, write, fd);
	fprintf(fd, "\n");
	vect.L1 = best_overshoot[0]; vect.L2 = best_overshoot[1]; vect.L3 = best_overshoot[2];
	simu_mcc(&vect, &val_out, write, fd);
	/*printf("%.3f - %.3f\n", val_out.overshoot, val_out.static_error);*/
	fprintf(fd, "\n");

	fclose(fd);
	free(datafile);
	if ( strcmp(argv[1], "simu") == 0 ) 
		plot_results("config_motor");
	else if ( strcmp(argv[1], "test") == 0 ) 
		plot_results("config");
	
	return 0;
}


//TODO optimisation
//TODO design
//TODO réduction quadratique
//TODO profile de comportement <=> perfs = f(input.conf)
//TODO intégration d'identification, de commande adaptative, robuste, (prédictive ??) ==> étude
//
//TODO Grace au profile de comportement, réglage des performances dynamique
//Ou alors en recherchant dans les résultats une configuration adequat
//Ou alors en mettant en forme l'affichage des perfs
//TODO Regler les bugs!!
//TODO Organiser le code avec une boîte réelle du processeur
