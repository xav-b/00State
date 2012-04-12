#include "opt_q.h"

int simu_mcc(conf* cpt_vect, output* cpt_out, int write, int i, FILE* fd) {
	int cpt = 0;
	float t = 0;
	int max_pos = 0;
	cp = 0;

	for (cpt=0; cpt < 2; cpt++) {
		cmd_i[cpt] = 0;
		cpt_out->position[cpt] = 0;
		cpt_out->speed[cpt]=0;
	}
	cpt = 0;

	do {
		if (cpt_vect->CP_FLAG) {
			cp = sin(cpt_out->position[0] * REF);
			/*printf("[DEBUG] Computing load perturbation [%2.2f]\n", cp*cpt_vect->LENGTH*cpt_vect->WEIGTH);*/
		}
		else
			cp = 0;

		get_pi(cpt_vect->CONSIGN, cpt_out, cpt_vect, cp, t);	 //compute cmd_i[0] of current loop
		mcc_model(cpt_out, cpt_vect, t);		 				//compute output[0]

		t += cpt_vect->DELTA;
	/*  increment indexes for next loop  */
		cpt_out->speed[1] = cpt_out->speed[0];
		cpt_out->position[1] = cpt_out->position[0];
		cmd_i[1] = cmd_i[0];
		
		if (fabs(cpt_out->position[0] - cpt_vect->CONSIGN) < (0.05*cpt_vect->CONSIGN))
			   cpt++;
		if (cpt == 60) {
			cpt_out->tm = t-(cpt*cpt_vect->DELTA);
			printf("[DEBUG][%d] Settled in %2.2f\n", i, cpt_out->tm);
		}		

		if (cpt_out->position[0] > max_pos)
			max_pos = cpt_out->position[0];

		if (write) {
			cpt_out->position[0] = cpt_out->position[0] * WN / REF;
			cpt_out->speed[0] = cpt_out->speed[0];	// WN
			fprintf(fd, "%5.4f\t%5.4f\t%5.4f\t%2.2f\n", t, cpt_out->position[0], cpt_out->speed[0], cp);
		}
		
	} while (t < cpt_vect->STOP); 
	cpt_out->static_error = fabs(cpt_out->position[0] - cpt_vect->CONSIGN)*100*WN/REF;
	cpt_out->overshoot = fabs(max_pos - cpt_vect->CONSIGN)*100*WN/REF;
	printf("%f\n", max_pos);

	return 0;
}

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
	int write = 0;
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
				
				simu_mcc(&vect, &val_out, write, i, fd);
				i++;

				/*fprintf(fd, "%d\t%4.2f\t%2.2f\t%2.2f\t%2.2f\t%2.2f\t%2.2f\n", i, val_out.overshoot, val_out.tm, val_out.static_error, vect.L1, vect.L2, vect.L3);*/

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

	         }
			 vect.L3 = L3_save;
	     }
		 vect.L2 = L2_save;
	 }
	printf("\nBest compromise: %f [%f %f %f]\n", best_all[3], best_all[0], best_all[1], best_all[2]);
	printf("Best tm: %f [%f %f %f]\n", best_tm[3], best_tm[0], best_tm[1], best_tm[2]);
	printf("Best overshoot: %f [%f %f %f]\n\n", best_overshoot[3], best_overshoot[0], best_overshoot[1], best_overshoot[2]);
	
    write = 1;
	vect.L1 = best_all[0]; vect.L2 = best_all[1]; vect.L3 = best_all[2];
	simu_mcc(&vect, &val_out, write, i, fd);
	fprintf(fd, "\n");
	vect.L1 = best_tm[0]; vect.L2 = best_tm[1]; vect.L3 = best_tm[2];
	simu_mcc(&vect, &val_out, write, i, fd);
	fprintf(fd, "\n");
	vect.L1 = best_overshoot[0]; vect.L2 = best_overshoot[1]; vect.L3 = best_overshoot[2];
	simu_mcc(&vect, &val_out, write, i, fd);
	fprintf(fd, "\n");

	fclose(fd);
	free(datafile);
	plot_results();
	
	return 0;
}
