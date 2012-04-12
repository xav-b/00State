#include "opt_q.h"


int main(int argc, char *argv[]) {
	//mtrace();
	//export MALLOC_TRACE=memory.log
	float max_pos=0, t=0, settled_time = 0;
	float L2_save = 0, L3_save = 0;
	int cpt = 0;
	float consign_r = 0;
	int i = 0, k = 0;
	float best_all[4] = {0,0,0,1000};
	float best_tm[4] = {0,0,0,1000};
	float best_overshoot[4] = {0,0,0,1000};
	char *datafile;
	FILE* fd;
	output val_out;
	conf vect;

	printf("\n-----   Numeric simulation for MCC command   ------\n");
	
	init(&val_out, &vect);		//Global variables initialisation
	consign_r = vect.CONSIGN * WN / REF;
	L2_save = vect.L2;
	L3_save = vect.L3;
	
	datafile = (char *)malloc(20);
	strcpy(datafile, "./results");
	printf("[DEBUG] datafile @  %p: \'%s\'\n", datafile, datafile);
	fd = fopen(datafile, "w");
	
	while (vect.L1 < vect.L1_MAX) {
	    vect.L1 += vect.L1_PAS;
	    while (vect.L2 < vect.L2_MAX) {
	        vect.L2+=vect.L2_PAS;
	        while (vect.L3 < vect.L3_MAX) {
	            vect.L3+=vect.L3_PAS;
				printf("[DEBUG] New simulation [%2.2f - %2.2f - %2.2f]\n", vect.L1, vect.L2, vect.L3);
				do {
					if (vect.CP_FLAG) {
						cp = sin(val_out.position[0] * REF);
						/*printf("[DEBUG] Computing load perturbation [%2.2f]\n", cp*vect.LENGTH*vect.WEIGTH);*/
					}
					else
						cp = 0;

					get_pi(vect.CONSIGN, &val_out, &vect, cp, t);	 //compute cmd_i[0] of current loop
					mcc_model(&val_out, &vect, t);		 				//compute output[0]

					t += vect.DELTA;
				/*  increment indexes for next loop  */
					val_out.speed[1] = val_out.speed[0];
					val_out.position[1] = val_out.position[0];
					cmd_i[1] = cmd_i[0];
					
					/*val_out.position[0] = val_out.position[0] * WN / REF;
					*val_out.speed[0] = val_out.speed[0];	// WN
					*
					*fprintf(fd, "%5.4f\t%5.4f\t%5.4f\t%2.2f\n", t, val_out.position[0], val_out.speed[0], cp);
					 */
					
					if (fabs(val_out.position[0] - vect.CONSIGN) < (0.05*vect.CONSIGN))
						   cpt++;
					if (cpt == 60) {
						val_out.tm = t-(cpt*vect.DELTA);
						printf("[DEBUG][%d] Settled in %2.2f\n", i, val_out.tm);
					}		
					if (val_out.position[0] > max_pos)
						max_pos = val_out.position[0];

				} while (t < vect.STOP); 
				i++;
				val_out.static_error = fabs(val_out.position[0] - vect.CONSIGN)*100*WN/REF;
				val_out.overshoot = fabs(max_pos - vect.CONSIGN)*100*WN/REF;

				fprintf(fd, "%d\t%4.2f\t%2.2f\t%2.2f\t%2.2f\t%2.2f\t%2.2f\n", i, val_out.overshoot, val_out.tm, val_out.static_error, vect.L1, vect.L2, vect.L3);

				if (val_out.static_error < vect.STATIC_TOLERANCE) {
					val_out.compromise = val_out.static_error + val_out.tm;
					if (val_out.compromise < best_all[3])  {
						best_all[0] = vect.L1; 
						best_all[1] = vect.L2; 
						best_all[2] = vect.L3; 
						best_all[3] = val_out.compromise;
					}
					if (val_out.tm < best_tm[3])  {
						best_tm[0] = vect.L1; 
						best_tm[1] = vect.L2; 
						best_tm[2] = vect.L3; 
						best_tm[3] = val_out.tm;
					}
					if (val_out.overshoot < best_overshoot[3])  {
						best_overshoot[0] = vect.L1; 
						best_overshoot[1] = vect.L2; 
						best_overshoot[2] = vect.L3; 
						best_overshoot[3] = val_out.overshoot;
					}
				}
				else 
					printf("[!][%d] Wrong configuration: %f [%f %f %f]\n", i, val_out.static_error, vect.L1, vect.L2, vect.L3);

				settled_time = 0;
				max_pos = 0;
				cpt = 0;
				for (k=0; k < 2; k++) {
					cmd_i[k] = 0;
					val_out.position[k] = 0;
					val_out.speed[k]=0;
				}
				t = 0;
				cp = 0;
	         }
			 vect.L3 = L3_save;
	     }
		 vect.L2 = L2_save;
	 }
	printf("\nBest compromise: %f [%f %f %f]\n", best_all[3], best_all[0], best_all[1], best_all[2]);
	printf("Best tm: %f [%f %f %f]\n", best_tm[3], best_tm[0], best_tm[1], best_tm[2]);
	printf("Best compromise: %f [%f %f %f]\n\n", best_overshoot[3], best_overshoot[0], best_overshoot[1], best_overshoot[2]);
	 
	fclose(fd);
	free(datafile);
	plot_results();
	
	return 0;
}
