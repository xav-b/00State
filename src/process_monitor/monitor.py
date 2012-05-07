#!/usr/bin/python
# -*- coding: utf8 -*-

"""==================================================
 * 	Filename:
 *	
 *	Description:
 *
 *	Version:
 * 	Created:
 * 	Compiler:
 *
 *	Author:
=================================================="""

import sys
import math
import matplotlib.pyplot as plt
import numpy as np

def check_iq(var, format, fd, tour, name):
	iq22_max = 350
	iq29_max = 4
	if (format == 22):
		format = iq22_max
	elif (format == 29):
		format == iq29_max
	else:
		print '[DEBUG - %g] Wrong format:	%s | %f' % (tour, name, format)
	if (abs(float(var)) > format):
		print '[DEBUG - %g] Format overflow:	%s | %f | %f' % (tour, name, var, format)
		return 1
	return 0

if (sys.argv[1] == 'h') or (len(sys.argv) < 2):
	print 'argv[1] = consigne en volt'
	print 'argv[2] = -4 < ThetaR < 4'
	sys.exit('help finish')

fd_out = open('check.dat', 'w')
# compteur de tour

# division de la fréquence de fonctionnement par 160

# Intégration du modèle de la MCC et simulation du manoeuvrage manuel du rotor
virtual = 1
test = 1 - virtual
# Paramètre de la correction
L1 = 20.6028
L2 = 76.4
L3 = 3.16228
DELTA = 0.01
L = 1
M = 0
CN = 17.8
# Initialisation
cmd_i0 = 0
position1 = 0
speed1 = 0
int_return = 0
cpt_tour = 0
cpt = 0
# Consigne réelle en volt entre 0 et 10 (1 tour)
ConsigneExt1 = float(sys.argv[1])
# Numérisation
ConsigneExt1 = ConsigneExt1 / 10

if (virtual == 0):
	# Relevé de la position entre -4(-pi) et 4(pi)
	ThetaR = float(sys.argv[2])
	while (ThetaR > 4):
		ThetaR -= 8
		cpt_tour += 1
	ThetaR /= 8
	position1 = ThetaR * 0.04  	# entre -0.5 et 0.5 puis réduit
						   		# le 0 est à l'horizontale à présent
	stop = 1

if (virtual or test):
	stop = 50
	position0 = np.zeros(stop)
	speed0 = np.zeros(stop)
	cmd_i1 = np.zeros(stop)

# Observation des variables intermédiaires
t = np.linspace(0,stop,stop)
Consigne_red_l2 = np.zeros(stop) 	#iq22
position1_return = np.zeros(stop) 	#iq22
speed1_return = np.zeros(stop) 		#iq22
pre_int_return = np.zeros(stop) 	#iq29
int_return = np.zeros(stop) 		#iq29
int_return_l3 = np.zeros(stop) 		#iq22

# Calcul de WR

# Réduction de la consigne (0.04 = 6.28 / 157 <=> 0.04 = 2*PI/WN)
Consigne_red_temp = ConsigneExt1 - cpt_tour
check_iq(Consigne_red_temp, 29, fd_out, cpt, Consigne_red_temp)
Consigne_red = Consigne_red_temp * 0.04
fd_out.write('Consigne reduite:	%f' % Consigne_red)

for k in range(0,stop):
	cpt += 1
	fd_out.write('tour: 	%g\n' % cpt)
	if (test):
		position1 += 0.001
	# Calcul de la charge
	cp = np.sin(position1 * 6.28) # plus valide
	check_iq(cp*M*L/CN, 22, fd_out, cpt, 'cp')
	fd_out.write('Couple de la charge réduite:	%f\n' % (cp * M * L / CN ))

	# Coeff L2 sur la consigne
	Consigne_red_l2[k] = L2 * Consigne_red
	fd_out.write('Consigne reduite * L2:	%f\n' % Consigne_red_l2[k])
	check_iq(Consigne_red_l2[k], 22, fd_out, cpt, 'Consigne_red_l2[k]')

	# Boucle de position et soustraction à la consigne 
	position1_return[k] = L2 * position1
	fd_out.write('position * L2:	%f\n' % position1_return[k])
	check_iq(position1_return[k], 22, fd_out, cpt, 'position1_return[k]')
	Consigne_red_l2[k] -= position1_return[k]
	fd_out.write('Consigne reduite - retour en position:	%f\n' % Consigne_red_l2[k])
	check_iq(Consigne_red_l2[k], 22, fd_out, cpt, 'Consigne_red_l2[k]')

	# Boucle de vitesse et soustraction à la consigne
	speed1_return[k] = L1 * speed1
	fd_out.write('WR * L1:	%f\n' % speed1_return[k])
	check_iq(speed1_return[k], 22, fd_out, cpt, 'speed1_return[k]')
	Consigne_red_l2[k] -= speed1_return[k]
	fd_out.write('Consigne réduite moins retour vitesse:	%f\n' % Consigne_red_l2[k])
	check_iq(Consigne_red_l2[k], 22, fd_out, cpt, 'Consigne_red_l2[k]')
	
	# Boucle d'intégration
	pre_int_return[k] = position1 - Consigne_red
	fd_out.write('pre intégration: %f\n' % pre_int_return[k])
	check_iq(pre_int_return[k], 29, fd_out, cpt, 'pre_int_return[k]')
	int_return[k] += DELTA * pre_int_return[k]
	fd_out.write('intégré: %f\n' % int_return[k])
	check_iq(int_return[k], 29, fd_out, cpt, 'int_return[k]')
	int_return_l3[k] = L3 * int_return[k] - (cp * 9.8 * M * L / CN)
	fd_out.write('intégrateur * L3: %f' % int_return_l3[k])
	check_iq(int_return_l3[k], 22, fd_out, cpt, 'int_return_l3[k]')
	vartemp22_1 = Consigne_red_l2[k] - int_return_l3[k]
	fd_out.write('Consigne réduite moins retour intégrateur:	%f\n' % vartemp22_1)
	check_iq(vartemp22_1, 22, fd_out, cpt, 'vartemp22_1')
	# Saturation de la variable qui peut s'accumuler
	if (vartemp22_1 > 1):
		print '[DEBUG - %g] Vartemp22_1 positive saturation:	%f' % (cpt, vartemp22_1)
		vartemp22_1 = 1
	if (vartemp22_1 < -1):
		print '[DEBUG - %g] Vartemp22_1 negative saturation:	%f' % (cpt, vartemp22_1)
		vartemp22_1 = -1

	# Affectation à la commande
	cmd_i0 = vartemp22_1
	fd_out.write('La même après première saturation à (-)1 (cmd_i0):	%f' % cmd_i0)
	check_iq(cmd_i0, 29, fd_out, cpt, 'cmd_i0')
	# SAturation d ela commande en couple à 25% du couple nominal
	if (cmd_i0 > 0.25):
		print '[DEBUG - %g] Command positive saturation:	%f' % (cpt, cmd_i0)
		cmd_i0 = 0.25
	elif (cmd_i0 < -0.25):
		print '[DEBUG - %g] Command negative saturation:	%f' % (cpt, cmd_i0)
		cmd_i0 = -0.25
	fd_out.write('La même après saturation à (-)0.25 (cmd_i0):	%f' % cmd_i0)
	fd_out.write('\n')

	if (test):
		position0[k] = position1
		cmd_i1[k] = cmd_i0
		speed0[k] = Consigne_red

	if (virtual):
		speed0[k] = (cmd_i1[k-1] * 0.0138) + (speed1 * 0.986)
		check_iq(speed0[k], 29, fd_out, cpt, 'speed0[k]')
		position0[k] = (speed1 * 0.01) + position1
		check_iq(position0[k], 29, fd_out, cpt, 'position0[k]')

		speed1 = speed0[k]
		position1 = position0[k]
		cmd_i1[k] = cmd_i0

# Affichage du résultat
print
print 'Position:	', position1
print 'Consigne:	', ConsigneExt1, 'tour(s)'
print 'Smart consigne:	', Consigne_red_temp
print 'Debordements:	', cpt_tour
print 'Commande:	', cmd_i0

if (virtual or test):
	plt.figure(1)
	#plt.subplot(221)
	plt.annotate('ThetaR', xy=(stop/2, position0[stop/2]), xytext=(3, 1.5),
			     arrowprops=dict(facecolor='black', shrink=0.05),)
	plt.title('Theta, wr, commande')
	plt.plot(t, position0, 'r', t, speed0, 'g', t, cmd_i1, 'b')
	plt.figure(2)
	plt.subplot(211)
	plt.title('Retours en position, vitesse et integrateur')
	plt.plot(t, position1_return, 'r', t, speed1_return, 'b', t, int_return_l3)
	plt.subplot(212)
	plt.title('Consigne reduitexl2, retour integrable, retour integre')
	plt.plot(t, Consigne_red_l2, 'r', t, pre_int_return, 'b', t, int_return)
	plt.show()
