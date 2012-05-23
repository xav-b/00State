/*
 *Usually usage function
 */
void usage (char* prog_name, int exit_code) {
    printf( "Usage: %s <simu|test>\n", prog_name );
    exit(exit_code);
}

// This function print an error msg befor exiting
void fatal(char *msg) {
        char error_msg[100];
         
        strcpy(error_msg, "[!!] Fatal error, ");
        strncat(error_msg, msg, 83);
        perror(error_msg);
        exit(-1);
}       
 
// This function extends malloc and control errors
void *ec_malloc(unsigned int size) {
        void *ptr;
        ptr = malloc(size);
        if (ptr == NULL)
                fatal("In ec_malloc() on memory allocation");
        return ptr;
}   
