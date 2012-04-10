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
import matplotlib.pyplot as plt
import numpy as np

if (sys.argv[1] == 'h') or (len(sys.argv) < 2):
	print 'argv[1] = consigne en volt'
	print 'argv[2] = -4 < ThetaR < 4'
	sys.exit('help finish')

fd_out = open('check.dat', 'w')
iq22_max = 350
iq29_max = 4
# compteur de tour

# division de la fréquence de fonctionnement par 160

# Intégration du modèle de la MCC et simulation du manoeuvrage manuel du rotor
virtual = 0
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
Consigne_red = Consigne_red_temp * 0.04
print 'Consigne réduite:	', Consigne_red

for k in range(0,stop):
	if (test):
		position1 += 0.001
	# Calcul de la charge
	cp = np.sin(position1 * 6.28) # plus valide

	# Coeff L2 sur la consigne
	Consigne_red_l2[k] = L2 * Consigne_red
	print 'Consigne réduite * L2: ', Consigne_red_l2[k]
	# Boucle de position et soustraction à la consigne 
	position1_return[k] = L2 * position1
	print 'position * L2: ', position1_return[k]
	Consigne_red_l2[k] -= position1_return[k]
	print 'Consigne réduite moins retour position:	', Consigne_red_l2[k]
	# Boucle de vitesse et soustraction à la consigne
	speed1_return[k] = L1 * speed1
	print 'L1 * speed: ', speed1_return[k]
	Consigne_red_l2[k] -= speed1_return[k]
	print 'Consigne réduite moins retour vitesse:	', Consigne_red_l2[k]
	# Boucle d'intégration
	pre_int_return[k] = position1 - Consigne_red
	print 'pre intégration: ', pre_int_return[k]
	int_return[k] += DELTA * pre_int_return[k]
	print 'intégré: ', int_return[k]
	int_return_l3[k] = L3 * int_return[k] - (cp * 9.8 * M * L / CN)
	print 'intégrateur * L3: ', int_return_l3[k]
	vartemp22_1 = Consigne_red_l2[k] - int_return_l3[k]
	print 'Consigne réduite moins retour intégrateur:	', vartemp22_1
	# Saturation de la variable qui peut s'accumuler
	if (vartemp22_1 > 1):
		vartemp22_1 = 1
	if (vartemp22_1 < -1):
		vartemp22_1 = -1

	# Affectation à la commande
	cmd_i0 = vartemp22_1
	print 'La même après première saturation à (-)1 (cmd_i0):	', cmd_i0
	# SAturation d ela commande en couple à 25% du couple nominal
	if (cmd_i0 > 0.25):
		cmd_i0 = 0.25
	elif (cmd_i0 < -0.25):
		cmd_i0 = -0.25
	print 'La même après saturation à (-)0.25 (cmd_i0):	', cmd_i0
	print

	if (test):
		position0[k] = position1
		cmd_i1[k] = cmd_i0
		speed0[k] = Consigne_red

	if (virtual):
		speed0[k] = (cmd_i1[k-1] * 0.0138) + (speed1 * 0.986)
		position0[k] = (speed1 * 0.01) + position1

		speed1 = speed0[k]
		position1 = position0[k]
		cmd_i1[k] = cmd_i0

# Affichage du résultat
print
print 'position:	', position1
print 'consigne:	', ConsigneExt1, 'tour(s)'
print 'smart consigne:	', Consigne_red_temp
print 'debordements:	', cpt_tour
print 'commande:	', cmd_i0

if (virtual or test):
	plt.figure(1)
	#plt.subplot(221)
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
