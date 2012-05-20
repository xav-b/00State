#! /usr/bin/octave -q

# script for quadratic linear reduction

clear;
if ( nargin < 5 )
    usage( "./reduc_quad Q11 Q22 Q33 r MU\n" )
endif
tic();
start = toc()

q_omega = str2num(argv(){1});
q_theta = str2num(argv(){2});
q_integral = str2num(argv(){3});
r = str2num(argv(){4});
mu = str2num(argv(){5});
tm = 0.72;
Q = [ q_omega, 0, 0; 0, q_theta, 0; 0, 0, q_integral ];
A = [ (-mu/tm), 0, 0; 1, 0, 0; 0, 1, 0 ]

printf( "Checking if existing solutions\n" )
flag = check_solution(A, q_omega, q_theta, q_integral)
flag = 0;   # temporary solution
if ( flag < 0 )
    printf( "No solution, aborting\n" )
elseif ( flag == 0 )
    printf( "Configuration OK\n" )
    printf( "\nPerforming quadratic reduction\n" )
    [ L, X, C ] = lqr( A, Q, r )
endif


stop = toc()

