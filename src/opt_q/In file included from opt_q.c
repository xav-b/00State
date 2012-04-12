/*==================================================
 * 	Filename:
 *	
 *	Description:
 *
 *	Version:
 * 	Created:
 * 	Compiler:
 *
 *	Author:
==================================================*/

#include <stdio.h>
#include <stdlib.h>

void f_start(void) __attribute__((constructor(101)));
void f_exit(void) __attribute__((destructor(101)));

void f_start(void) {
	printf("On start\n");
}

void f_exit(void) {
	printf("On exit\n");
}

int main(int argc, char *argv[]) {
	return(0);
}


