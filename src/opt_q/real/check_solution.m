## Usage: check_solution( A, q_omega, q_theta, q_integral )
##
## Description
## Input:
## Output:

function flag = check_solution( A, q_omega, q_theta, q_integral )
	
    flag = 0;
	if ( nargin < 4)
		usage( "reduc_quad( A, q_omega, q_theta, q_integral )");
        flag = -1;
	endif

	Q = [ q_omega, 0, 0; 0, q_theta, 0; 0, 0, q_integral ];
	# Verification de l'existence de solutions
	H = [ sqrt(q_omega), 0, 0; 0, sqrt(q_theta), 0; 0, 0, sqrt(q_integral) ];
	#TODO détermination de H vérifiant Q = t.' * H
	HA = H.*A;

	if ( rank(A) == rank([H, HA]))
#		printf( "Ranks equal\n" );
        flag = 0;
	else
		##error( "! Different ranks, no solution" );
 #       printf( "Ranks not equal\n" )
        flag = -1;
	endif
	
endfunction
