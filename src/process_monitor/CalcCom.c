/*********************************************************************
 
La fonction "main" utilise les routines suivantes :
- InitSysCtrl()
- InitGpio(1, 0)
- InitPieCtrl()
- InitPieVectTable()
- InitEv()
- InitSpi(1)						
- InitSci(0)						
- InitPwm(DemiPeriode, 0, ActiveModRSCE)
- ActivTempsMorts(TempsMorts)
- InitAdc(Mode_debug)
- ActivTimer1_ISR(ActiveT1PINT_ISR, ActiveT1CINT_ISR, ActiveT1UFINT_ISR);
- ActivTimer2_ISR(ActiveT2PINT_ISR, ActiveT2CINT_ISR);
- ActivTimer3_ISR(ActiveT3PINT_ISR, ActiveT3CINT_ISR);
- ActivTimer4_ISR(ActiveT4PINT_ISR, ActiveT4CINT_ISR);
- EnableInterrupts();
- AnaAcquis(Calibrage, Calibrate, &Consigne, &Temp, &I0, &U0, &Omega, &Ia, &Ic)
- CalcCom()

La fonction "CalcCom"  utilise les routines suivantes :
- AnaAcquis(Calibrage, Calibrate, &Consigne, &Consigne2, &I0, &U0, &Omega, &Ia, &Ic);
- ControleGrandAna(Ia, Ib, Ic, U0)
- ActivTempsMorts(TempsMorts);
- ActivPWM(PWM, 0, DepartArretPWM);
- SetPWM(Vref_a, Kmod, 1, _IQ29(DutyMin), _IQ29(DutyMax));
- AnalogOut(_IQ29(0), 1);

=====================================================================================
 History:
-------------------------------------------------------------------------------------

	17/04/09		V10		BB		Version initiale équivalente à Modele_V10
	
**********************************************************************/

#include	"DSP281x_Device.h"     // DSP281x Headerfile Include File
#include 	"DSP281x_Examples.h"   // DSP281x Examples Include File
#include 	"IQmathLib.h"

#include	"Commande_asm.h"

// Constantes symboliques
#include	"Constantes.h"
#include	"IOnum.h"
#include	"MAEcoef.h"
#include	"ConstantesVectRot.h"
#include	"Calculs.h"

// Fonctions de calcul
#include	"Init.h"
#include	"Acquis.h"
#include	"SortieAna.h"
#include	"SerialBus.h"
#include	"PWM.h"
#include	"SortieModRSCE.h"
#include	"Controle.h"
#include	"Transform.h"
#include	"RefSinus.h"
#include	"TimerCmpInt.h"
// ***************** VERSION CARTE COMMANDE  *************************
#if TargetCard == 1
#include	"CodeurOpt.h"
#endif

//Déclaration des variables

//#define INIT_COMPTEUR 	32000.0
_iq29	consign;
_iq29 	overflow;
_iq29 	max_pos;
_iq29 	t;
_iq29 	current;
_iq29 	position0;
_iq29 	position1;
_iq29 	position2;
_iq29 	speed0;
_iq29 	speed1;
_iq29 	speed2;
_iq29 	cmd_i0;
_iq29 	cmd_i1;
_iq29 	cmd_i2;
_iq29 	return_int;
_iq29 	return_int_temp;
_iq29 	mcc_coeff1;
_iq29 	mcc_coeff2;
_iq29 	delta;
_iq29 	l1;
_iq22 	l2;
_iq29 	l3;
_iq29 	Test;
_iq29 	Double;
_iq22 	coeff_ret_temp;
_iq22 	cmd_i0_temp;
_iq27 	cmd_i0_temp2;
_iq29 	Consigne_red_temp;
_iq29	Consigne_red;
_iq22 	coeff_ret_temp_1;
_iq22   vartemp22_1;
_iq22   vartemp22_2;
_iq22   vartemp22_3;
_iq29   var_temp29;
_iq29   vartemp29_2;
_iq29   vartemp29_3;
_iq22 	Consigne_red_l2;
_iq22	position1_return;
_iq22	speed1_return;
_iq29	pre_int_return;
_iq22	int_return_l3;
_iq29	int_return;
_iq22	cp;
_iq29	cpt_tour;
_iq22	DeltaThetaR_22;
int 	compteur_dem;
int 	magnet;
int		count;
int 	load;
int		PupitreMarcheArret;
int		virtual;


// déclaration de prototypes utilisés
interrupt void AdcIsr(void);		// Remplace ADCINT_ISR dans DSP281x_DefaultIsr.c

interrupt void SpiRXaIsr(void);		// Remplace SPIRXINTA_ISR dans DSP281x_DefaultIsr.c
interrupt void SpiTXaIsr(void);		// Remplace SPITXINTA_ISR dans DSP281x_DefaultIsr.c

interrupt void SciRXaIsr(void);		// Remplace SCIRXA_ISR dans DSP281x_DefaultIsr.c
interrupt void SciTXaIsr(void);		// Remplace SCITXA_ISR dans DSP281x_DefaultIsr.c

interrupt void T1PeriodeIsr(void);	// Remplace T1PINT_ISR dans DSP281x_DefaultIsr.c
interrupt void T1CompareIsr(void);	// Remplace T1CINT_ISR dans DSP281x_DefaultIsr.c
interrupt void T1UnderFlowIsr(void);// Remplace T1UFINT_ISR dans DSP281x_DefaultIsr.c
interrupt void T2PeriodeIsr(void);	// Remplace T1PINT_ISR dans DSP281x_DefaultIsr.c
interrupt void T2CompareIsr(void);	// Remplace T1CINT_ISR dans DSP281x_DefaultIs
interrupt void T3PeriodeIsr(void);	// Remplace T1PINT_ISR dans DSP281x_DefaultIsr.c
interrupt void T3CompareIsr(void);	// Remplace T1CINT_ISR dans DSP281x_DefaultIs
interrupt void T4PeriodeIsr(void);	// Remplace T1PINT_ISR dans DSP281x_DefaultIsr.c
interrupt void T4CompareIsr(void);	// Remplace T1CINT_ISR dans DSP281x_DefaultIsr.c
// ***************** VERSION CARTE COMMANDE  *************************
#if TargetCard == 1
interrupt void QepIsr(void);		// Remplace CAP3INT_ISR dans DSP281x_DefaultIsr.c
#endif


// Variables diverses
/* 				ATTENTION 
l'initialisation dans la déclaration ne fonctionne
plus lorsque le programme est chargé en flash.
Ne pas oublier d'initialiser les variables au début du programme*/
int			Calibrage;

Uint16		DepartArretPWM;				// Condition de départ des PWM
Uint16		Defaut;						// Condition de départ des PWM
_iq29		Kmod;						// Gain du modulateur
// Tension de commande envoyé au PWM
long		compteur_demarrage;			//compteur pour saturer les tensions de référence au démarrage (pendant deux secondes)
_iq29		LimiteV;		 			//valeur de saturation pour le démarrage
_iq29		Commande, Commande_prec;
_iq29		temp_pos;

// Choix des sorties analogiques
int			SortieCNA1;					// Affichage sur la sortie CNA1
int			SortieCNA2;					// Affichage sur la sortie CNA2
// ***************** VERSION CARTE COMMANDE  *************************
#if TargetCard == 1
// Sur la carte finale 2 sortie analogiques supplémentaires sont disponibles.
int			SortieCNA3;					// Affichage sur la sortie CNA3
int			SortieCNA4;					// Affichage sur la sortie CNA4
#endif


int			ActiveT1CINT_ISR;			/* Bit d'activation de l'interruption
										de comparaison du Timer1*/
int			ActiveT1PINT_ISR;			/* Bit d'activation de l'interruption
										de demi-période du Timer1*/
int			ActiveT1UFINT_ISR;			/* Bit d'activation de l'interruption
										de fin de période du Timer1*/
int			ActiveT2CINT_ISR;			/* Bit d'activation de l'interruption
										de comparaison du Timer2*/
int			ActiveT2PINT_ISR;			/* Bit d'activation de l'interruption
										de demi-période du Timer2*/
int			ActiveT3CINT_ISR;			/* Bit d'activation de l'interruption
										de comparaison du Timer3*/
int			ActiveT3PINT_ISR;			/* Bit d'activation de l'interruption
										de demi-période du Timer3*/
int			ActiveT4CINT_ISR;			/* Bit d'activation de l'interruption
										de comparaison du Timer4*/
int			ActiveT4PINT_ISR;			/* Bit d'activation de l'interruption
										de demi-période du Timer4*/

Uint16		FlagEndTe;
Uint16		EtatHaut;

// Conversions analogiques numériques
Uint16		ADCFlag;


// Liaison SPI resolveur
Uint16		SpiRXFlag;

// Liaison UART
// Flags
Uint16		SciRXFlag;					// Flag indiquant qu'une trame a été reçue
Uint16		SciTXFlag;					// Flag indiquant qu'une trame a été envoyée
Uint16		SciEnvoie;					// Flag indiquant si une donnée doit être
										// envoyée 0 --> pas d'envoie
										// 1 --> pas d'envoie mais préparation des
										// données
										// 2 --> envoie des données
										// une autre donnée (>0)
Uint16		SciRecept;					// Flag indiquant si la donnée reçue
										// correspond au protocole (=0) ou à
										// une autre donnée (>0)
Uint16		SciTXBit;					// Flag indiquant la trame à envoyer
Uint16		SciRXBit;					// Flag indiquant la trame reçue
Uint16		SciFlag;
// Reception
Uint16		DataReceived;				// Trame de 8 bits reçue non signée
int			DataReceivedSign;			// Mot complet de 16 bits reçue signé
// Tableau des données non signées comportant SciDataReceiveNb données à recevoir
// + 1 donnée initiale permettant d'assurer un protocole de transmission et de vérifier
// que la transmission s'est correctement déroulée. La donnée correspondant au ptotocole
// est l'élément 0.

int			DataToReceive[SciDataReceiveNb];
Uint16		IndexToReceive;			// Indice permettant de controler l'élément du tableau reçue
Uint16		SciReceivedEnd;

// Transmission
Uint16		DataToTransmit[SciDataTransmitNb];
Uint16		IndexToTransmit;			// Indice permettant de controler l'élément du tableau transmis

int			i_coeff;										
int			DelaiEnvoie;	



// ***************** VERSION CARTE COMMANDE  *************************
#if TargetCard == 1
// Instance a QEP interface driver 
Uint16		QepTOP0Flag;
_iq29		ThetaRCodOpt;
Uint16		DirRCodOpt;
#endif


// Acquisitions & Etalonnage
#if TargetCard == 0
// ***************** VERSION CARTE DEVELOPPEMENT  *************************
_iq29		Calibrate[7];
#else
// ***************** VERSION CARTE COMMANDE  *************************
// La carte finale dispose de 4 acquisitions supplémentaires.
_iq29		Calibrate[11];
#endif



/* DECLARATION DES VARIABLES POUR LA COMMANDE VECTORIELLE */
// indices pour les variables circulaires
int			IndiceFlux;
int			IndiceCourants;
int			IndiceTensions;
int			IndiceEstimFlux;
// compteur pour la correction du flux
int			BoucleFlux = NB_CYCLES_FLUX;
// Variables pour les calculs
_iq29		Id[NB_IDQ], Iq[NB_IDQ];
_iq29		IdRef[NB_IDQ], IqRef[NB_IDQ];
_iq29		IdErr[NB_IDQ], IqErr[NB_IDQ];
_iq29		Vd[NB_VDQ], Vq[NB_VDQ];
_iq29		PsiRef[NB_PSI], PsiR[NB_PSI];
_iq29		PsiErr[NB_PSI];
_iq29		Cref[NB_C];
_iq29		TetaPk;
_iq29		WR, WPk;

_iq29		Uc_sur_2;		// Uc sur 2 normalisée à Vsn = coefficient


_iq29		ThetaR[10];						// Position
_iq29		ThetaPk;
_iq29		DeltaThetaR;
_iq29		U0;							// Tension du bus continu
_iq29		I0;							// Courant côté continu
_iq29		Ia, Ib,	Ic;					// Courants sortant de l'onduleur
_iq29		Omega;
_iq29		Vref_a, Vref_b, Vref_c;		// Tensions moyenness à appliquer sur sur le convertisseur
_iq29		Vref_a_Opt, Vref_b_Opt, Vref_c_Opt;		// Modulation optimisee
_iq29		Vcrete;
_iq29		ConsigneExt1, ConsigneExt2, ConsigneExt3;
_iq29		ConsignePot;
_iq29		Temp;						// Variable inutilisée sauf pour passer un argument fictif
float		TempFloat;

int			i;							// indice de boucle


//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
void InitCalcCom(void)
{

//initialisation des variables 

	Test = _IQ29(0);
	Double = _IQ29(0);
	consign = _IQ29(0);
	overflow = _IQ29(0); 
	max_pos = _IQ29(0);
	t = _IQ29(0);
	current = _IQ29(0);
	position0 = _IQ29(0);
	position1 = _IQ29(0);
	position2 = _IQ29(0);
	speed0 = _IQ29(0);
	speed1 = _IQ29(0);
	speed2 = _IQ29(0);
	cmd_i0 = _IQ29(0);
	cmd_i1 = _IQ29(0);
	cmd_i2 = _IQ29(0);
	return_int = _IQ29(0);
	return_int_temp = _IQ29(0);
	coeff_ret_temp = _IQ22(0);
	mcc_coeff1 = 0.000013889;
	mcc_coeff2 = 0.999986;
	delta = 0.00001;
	cmd_i0_temp = _IQ22(0);
	cmd_i0_temp2 = _IQ27(0);
	Consigne_red_temp = _IQ28(0);
	Consigne_red = _IQ29(0);
	coeff_ret_temp_1 = _IQ22(0);
	vartemp22_1 = _IQ22(0);
	vartemp22_2 = _IQ22(0);
	vartemp22_3 = _IQ22(0);
	var_temp29 = _IQ29(0);;
	vartemp29_2 = _IQ29(0);;
	vartemp29_3 = _IQ29(0);;
	Consigne_red_l2 = _IQ22(0);
	position1_return = _IQ22(0);
	speed1_return = _IQ22(0);
	pre_int_return = _IQ29(0);
	int_return_l3 = _IQ22(0);
	int_return = _IQ29(0);
	cp = _IQ22(0);
	cpt_tour = _IQ29(0);
	DeltaThetaR_22 = _IQ22(0);
	compteur_dem = 0;
	magnet = 1;
	count = 0;
	load = 0;
	PupitreMarcheArret = 0;
	virtual = 0;
	
	
/*************************************************
			 Vérification des coefficients
			 calculés dans les "*.h"
**************************************************/
	TempFloat = Adapt_U0pe_Def;
	TempFloat = Adapt_Ipe_Def;
	TempFloat = Adapt_Cons_Def;
	TempFloat = Adapt_Omegape_Def;

		
/*************************************************
			 Initialiation des variables
**************************************************/
	DepartArretPWM = 0;					// Condition de départ des PWM
	Defaut = 0;							// Pas de défaut 
	Kmod = _IQ29(1);
	Commande = _IQ29(0);
	Commande_prec = _IQ29(0);	


// Initialisation des sorties analogiues
	SortieCNA1 = 0;						// ConsigneExt1
	SortieCNA2 = 0;						// ConsigneExt2
// ***************** VERSION CARTE COMMANDE  *************************
#if TargetCard == 1
// Sur la carte finale 2 sortie analogiques supplémentaires sont disponibles.
	SortieCNA3 = 0;						// ConsigneExt3
	SortieCNA4 = 0;						// ConsignPot
#endif

	
// Initialisation des autres grandeurs
	Ia = _IQ29(0);
	Ib = _IQ29(0);
	Ic = _IQ29(0);
	I0 = _IQ29(0);
	U0 = _IQ29(0);
	Vref_a = _IQ29(0);
	Vref_b = _IQ29(0);
	Vref_c = _IQ29(0);
	Vref_a_Opt = _IQ29(0);
	Vref_b_Opt = _IQ29(0);
	Vref_c_Opt = _IQ29(0);
	ConsigneExt1 = _IQ29(0);
	ConsigneExt2 = _IQ29(0);
	ConsigneExt3 = _IQ29(0);
	ConsignePot = _IQ29(0);
	Temp = _IQ29(0);
	IndiceFlux = 0;
	IndiceCourants = 0;
	IndiceTensions = 0;
	IndiceEstimFlux = 0;

/*** Initialisation des variables et des pointeurs ***/
	for (i=0; i<NB_PSI; i++)
	{
		PsiErr[i] = 0;
		PsiR[i] = 0;
	}
	for (i=0; i<NB_C; i++)
	{
		Cref[i] = 0;
		Cref[i] = 0;
	}
	for (i=0; i<NB_IDQ; i++)
	{
		IdRef[i] = 0;
		Id[i] = 0;
	}
	Uc_sur_2 = _IQ29(2);		// Uc sur 2 normalisée à Vsn = coefficient
	TetaPk = _IQ29(0);
	
	
// Liaison serie
// Initialisation des variables du port série
	SciRXFlag = 0;
	SciTXFlag = 0;
	SciEnvoie = 0;
	SciRecept = 0;
	SciTXBit = 0;
	SciRXBit = 0;
	SciFlag = 0;
// Reception
	DataReceived = 0;
	DataReceivedSign = 0;
	for ( i = 0 ; i <= SciDataReceiveNb ; i++ )
		DataToReceive[i] = 0;
	IndexToReceive = 0;			// Indice permettant de controler l'élément du tableau reçue
	SciReceivedEnd = 0;

// Transmission
	for ( i = 0 ; i <= SciDataTransmitNb ; i++ )
		DataToTransmit[i] = 0;
	IndexToTransmit = 0;			// Indice permettant de controler l'élément du tableau transmis

	DelaiEnvoie = 8000;


// Activation du mode de gestion de l'overflow
	SETOVM;


// Inhibition des interruptions de comparaison du timer1.
	ActiveT1CINT_ISR = 0;
// Inhibition des interruptions de demi-période du timer1.
	ActiveT1PINT_ISR = 0;
// Inhibition des interruptions de fin de période du timer1.
	ActiveT1UFINT_ISR = 0;
// Inhibition des interruptions de comparaison du timer2.
	ActiveT2CINT_ISR = 0;
// Inhibition des interruptions de demi-période du timer2.
	ActiveT2PINT_ISR = 0;
// Inhibition des interruptions de comparaison du timer3.
	ActiveT3CINT_ISR = 0;
// Inhibition des interruptions de demi-période du timer3.
	ActiveT3PINT_ISR = 0;
// Inhibition des interruptions de comparaison du timer4.
	ActiveT4CINT_ISR = 0;
// Inhibition des interruptions de demi-période du timer4.
	ActiveT4PINT_ISR = 0;


// Step 3. Re-map ISR functions:
/**************************************************
			INTERRUPTIONS UTILISEES
***************************************************/
// Interrupts that are used in this example are re-mapped to
// ISR functions found within this file.
	EALLOW;  // This is needed to write to EALLOW protected register
	PieVectTable.ADCINT = &AdcIsr;
	PieVectTable.SPIRXINTA = &SpiRXaIsr;
	PieVectTable.SPITXINTA = &SpiTXaIsr;
	PieVectTable.RXAINT = &SciRXaIsr;
	PieVectTable.TXAINT = &SciTXaIsr;
	PieVectTable.T1PINT = &T1PeriodeIsr;
	PieVectTable.T1CINT = &T1CompareIsr;
	PieVectTable.T1UFINT = &T1UnderFlowIsr;
	PieVectTable.T2PINT = &T2PeriodeIsr;
	PieVectTable.T2CINT = &T2CompareIsr;
	PieVectTable.T3PINT = &T3PeriodeIsr;
	PieVectTable.T3CINT = &T3CompareIsr;
	PieVectTable.T4PINT = &T4PeriodeIsr;
	PieVectTable.T4CINT = &T4CompareIsr;
// ***************** VERSION CARTE COMMANDE  *************************
#if TargetCard == 1
	PieVectTable.CAPINT3 = &QepIsr;
	EDIS;    // This is needed to disable write to EALLOW protected registers
#endif


// Step 4. Initalize GPIO:
/**************************************************
			ENTREES / SORTIES
***************************************************/ 
// I/O. Function is found in the Init.c file
#if TargetCard == 0
// ***************** VERSION CARTE DEVELOPPEMENT  *************************
/* Il est possible de choisir entre entrée et sortie pour Num1 et Num2 dans la carte
   initiale en accord avec la position des jumpers correspondants.
   Sortie Num 1 est configuré en sortie (1) (correspond à BitSynchroTe)
   Sortie Num 2 est configuré en sortie (0) (correspond à BitModRSCE)
*/
	InitGpio_ESME(1, 0);
#else
// ***************** VERSION CARTE COMMANDE  *************************
// La carte définitive dispose de 3 I/O Num qui sont par défaut des ENTREES
//	InitGpio_ESME(0, 0, 0, 0); 

/* 						ATTENTION
 Si les I/O Num sont utilisées comme sortie, alors il faut IMPERATIVEMENT
 s'assurer que les interrupteurs correspondant soit en position ARRET c'est
 à dire vers le HAUT  (ou vers le bas pour la carte PROTO qui est à l'envers)
 sous peine d'endommager la carte.
	void InitGpio_ESME(Uint16 SortieLed,
					   Uint16 ConfigLed_a0,
					   Uint16 ConfigLed_a1,
					   Uint16 ConfigLed_a2);
	SortieLed = 0 --> valeurs par défaut, les 3 sont des entrées
	SortieLed = 1 --> prend la valeur des 3 bits d'argument
		Entrée = 0
		Sortie = 1
*/
// On utilise le bit a2 comme indicateur du signe de la consigne. Il est donc
// configuré en sortie.
// a0 et a1 restent donc en entrées numériques 
//	InitGpio_ESME(1, 0, 0, 1);
	InitGpio_ESME();
#endif 



// Step 5. Initialize all the Device Peripherals:
/**************************************************
		FONCTIONS SUR EVENEMENTS
***************************************************/
// Initialize the Event Managers (FILE: Init.c)
	InitEv_ESME();


/**************************************************
			LIAISONS SERIE
***************************************************/
// Initialise le port série synchrone
/* Inhibition des interruptions de réception pour la 
mesure de position */
	InitSpi_ESME(SET_RESOLVEUR);
	SpiRXFlag = 0;



// Initialise le port série asynchrone (UART)
// Activation des interruptions de réception
	InitSci_ESME(SET_COM_PC2DSP, SET_COM_DSP2PC);
	SciRXFlag = 0;
	SciTXFlag = 0;						
  

/**************************************************
			PWM & ACQUISITIONS
***************************************************/
// Utilisation des PWM et non des vecteurs d'espace
// Pas de modulateur RSCE mais sorties CNA
	InitPwm_ESME(DemiPeriode_Def, SELECT_PWM_MODE, SET_MOD_RSCE);	// Initialise le module PWM (fichier: Init.c) 
	ActivTempsMorts(SET_DEAD_TIME);			// Activation des temps morts

	InitAdc_ESME(MODE_DEBUG);					// Initialize the PWM (FILE: Init.c)

/***************************************************/



/**************************************************
	UTILISATION DES INTERRUPTIONS DES TIMERS
***************************************************/
// Inhibition des interruptions de comparaison Timer1
	ActivTimer1_ISR(ActiveT1PINT_ISR, ActiveT1CINT_ISR, ActiveT1UFINT_ISR);
// Inhibition des interruptions du Timer2
	ActivTimer2_ISR(ActiveT2PINT_ISR, ActiveT2CINT_ISR);
// Inhibition des interruptions du Timer3
	ActivTimer3_ISR(ActiveT3PINT_ISR, ActiveT3CINT_ISR);
// Inhibition des interruptions du Timer4
	ActivTimer4_ISR(ActiveT4PINT_ISR, ActiveT4CINT_ISR);


 
/**************************************************
			CODEUR OPTIQUE
***************************************************/  
// ***************** VERSION CARTE COMMANDE  *************************
#if TargetCard == 1
// Initialise le codeur optique 
	if (SET_CODEUR_INC  == 1)
	{
    	InitQEP_ESME(SET_CODEUR_INC);
		QepTOP0Flag = 0;
		ThetaRCodOpt = _IQ29(0);
		DirRCodOpt = 0;
	}
#endif 


  	
	
/*************************************************
		Etalonnage des entrées analogique
*************************************************/
// Fait la moyenne de plusieurs acquisitions lorsque la puissance est coupée pour déterminer les Offsets
	Calibrate[0] = 0x0000;
	Calibrate[1] = 0x0000;
	Calibrate[2] = 0x0000;
	Calibrate[3] = 0x0000;	
	Calibrate[4] = 0x0000;	
	Calibrate[5] = 0x0000;	
	Calibrate[6] = 0x0000;
// ***************** VERSION CARTE COMMANDE  *************************
#if TargetCard == 1
// La carte finale dispose de 4 acquisitions supplémentaires.
	Calibrate[7] = 0x0000;	
	Calibrate[8] = 0x0000;	
	Calibrate[9] = 0x0000;	
	Calibrate[10] = 0x0000;
#endif

 	for (Calibrage=1; Calibrage <= CAN_CALIBRATION_OFF; Calibrage++)
 	{
#if TargetCard == 0
// ***************** VERSION CARTE DEVELOPPEMENT  *************************
// CAN0=Offset, CAN1 à CAN7 dans l'ordre svt  :
// ConsigneExt1, ConsigneExt2, I0, U0, Omega, Ia, Ic	
// Arguments : AnaAcquis
// Active Etalonnage, Tableau compensation, I0, U0, ConsigneExt1, Omega, &ConsigneExt3, &ConsigneExt2, &Temp)	
		
		AnaAcquis(Calibrage, Calibrate, &I0, &U0, &ConsigneExt1, &Omega, &ConsigneExt3, &ConsigneExt2, &Temp);
		
#else
// ***************** VERSION CARTE COMMANDE  *************************
// CAN11=Offset, CAN0 à CAN10 dans l'ordre svt :
// I0, U0, ConsigneExt1, Omega, ConsigneExt3, ConsigneExt2, Vc, Va, Ia, ConsignePot, Ic
// Arguments : AnaAcquis
// Active Etalonnage, Tableau compensation, I0, U0, ConsigneExt1, Omega, ConsigneExt3, ConsigneExt2, Vc, Va, Ia, ConsignePot, Ic	
		AnaAcquis(Calibrage, Calibrate, &I0, &U0, &ConsigneExt1, &Omega, &ConsigneExt3, &ConsigneExt2, &Temp, &Temp, &Ia, &ConsignePot, &Ic);
#endif
		WAITFORINT;
 	}


	CalcTime = 0;
	Defaut = 0;
	DepartArretPWM = 0;
	SETOVM;

} // end main()

//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!




//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
void CalcCom(void)
{
	TogCalcTime = 1;
	var_temp29 = ThetaR[9];
	vartemp29_2 = WR;

 /*****************************************************************
 Lecture des grandeurs analogiques & Adaptation à la pleine échelle
  des valeurs mesurées
 *****************************************************************/	
// ***************** VERSION CARTE DEVELOPPEMENT  *************************
#if TargetCard == 0
// CAN0=Offset, CAN1 à CAN7 dans l'ordre svt  :
// ConsigneExt1, ConsigneExt2, I0, U0, Omega, Ia, Ic	
	if (SET_RESOLVEUR == 1)
// Arguments : Resolver_Cde
// Tableau compensation, Position, ConsigneExt1, ConsigneExt2, I0, U0, Omega, Ia, Ib, Ic	
		Resolver_Cde(Calibrate, &ThetaR[9], &ConsigneExt1, &ConsigneExt2, &I0, &U0, &Omega, &Ia, &Ib, &Ic);
	else
// Arguments : AnaAcquis
// Active Etalonnage, Tableau compensation, ConsigneExt1, ConsigneExt2, I0, U0, Omega, Ia, Ic	
		AnaAcquis(CAN_CALIBRATION_OFF, Calibrate, &I0, &U0, &ConsigneExt1, &Omega, &ConsigneExt3, &ConsigneExt2, &Temp);

// ***************** VERSION CARTE COMMANDE  *************************
#else
// CAN11=Offset, CAN0 à CAN10 dans l'ordre svt :
// I0, U0, ConsigneExt1, Omega, ConsigneExt3, ConsigneExt2, Vc, Va, Ia, ConsignePot, Ic

// Arguments : AnaAcquis
// Active Etalonnage, Tableau compensation, I0, U0, ConsigneExt1, Omega, ConsigneExt3, ConsigneExt2, Vc, Va, Ia, ConsignePot, Ic	
		AnaAcquis(CAN_CALIBRATION_OFF, Calibrate, &I0, &U0, &ConsigneExt1, &Omega, &ConsigneExt3, &ConsigneExt2, &Temp, &Temp, &Ia, &ConsignePot, &Ic);


// Appel de la routine de calcul de la position à partir d'un codeur incrémental
	if (SET_CODEUR_INC == 1)
    	CodeurOptCalc(&ThetaRCodOpt, &DirRCodOpt);	

/*	
	if ( PupitreModeCde )
		Consigne = ConsigneExt1;
	else
		Consigne= ConsignePot;
*/		
				 
#endif


/*****************************************************************
 Adaptation à la pleine échelle des valeurs mesurées
******************************************************************/
// Si le resolveur est actif, la mise à l'échelle est effectuée dans
// la routine Resolver_Cde
	if (SET_RESOLVEUR == 0)
	{
// Tension du bus continu.
		U0 = _IQ29mpyIQX(U0, 29, _IQ28(Adapt_U0pe_Def), 28);

// Courant du bus continu.
		I0 = _IQ29rsmpy(I0, _IQ29(Adapt_I0pe_Def));

// Courants côté alternatif
		Ia = _IQ29rsmpy(Ia, _IQ29(Adapt_Ipe_Def));
		Ic = _IQ29rsmpy(Ic, _IQ29(Adapt_Ipe_Def));

// Calcul du courant Ib
		Ib = - Ia - Ic;
	}


/*****************************************************************
 Commande Vectorielle
******************************************************************/

// Décalage pour les variables circulaires
	for (i=1; i<NB_IDQ; i++)
	{
		Id[i-1] = Id[i];
		Iq[i-1] = Iq[i];
		IqRef[i-1] = IqRef[i];
		IdErr[i-1] = IdErr[i];
		IqErr[i-1] = IqErr[i];
	}
	for (i=1; i<NB_VDQ; i++)
	{
		Vd[i-1] = Vd[i];
		Vq[i-1] = Vq[i];
	}
	for (i=1; i<NB_PSI; i++)
	{
		PsiR[i-1] = PsiR[i];
	}
	for (i=1; i<10; i++)
	{
		ThetaR[i-1] = ThetaR[i];
	}

// Stockage de l'angle et calcul de la vitesse
	//ThetaR[9] = - ThetaRCodOpt;
	if (_IQ29rsmpy(ThetaR[9],ThetaR[0]) >= _IQ29(0))
	{
		asm(" clrc OVM ");
		DeltaThetaR = ThetaR[9] - ThetaR[0];
		asm(" setc OVM ");
		WR = _IQ29mpyIQX(DeltaThetaR, 29, _IQ24(7.71), 24);
	}





//******************************************************************
// Notre code
//******************************************************************

CLROVM;
DeltaThetaR = ThetaR[9] - ThetaR[8];
if (ThetaR[9] < _IQ29(-3.8) && ThetaR[7] > _IQ29(3.8))
	cpt_tour += _IQ29(0.25);
if (ThetaR[9] > _IQ29(3.8) && ThetaR[7] < _IQ29(-3.8))
	cpt_tour -= _IQ29(0.25);
SETOVM;

//vartemp29_2 = cpt_tour + (ThetaR[9] >> 1 + _IQ29(2)) >> 2;
count += 1;

if(count == 1) {
	count = 0;
	if (virtual == 0) {
		/*position1 = ThetaR[9]>>2;
		position1 =  _IQ29(0.5) + (position1>>1);
		position1 =  cpt_tour + (position1>>2);
		position1 = _IQsat(position1, _IQ29(1), _IQ29(-1));
		*/
		//position1 = _IQ29(0.5) + (ThetaR[9]>>3);
		position1 = ThetaR[9];
		speed1 = WR;
	}

	if (magnet == 0) {
		if (load)
			cp = _IQ22sin(_IQ22mpyIQX(position1, 29, _IQ22(6.28), 22));

		//Consigne_red_temp = ConsigneExt1 - cpt_tour;
		Consigne_red_temp = _IQmpy(ConsigneExt1, _IQ29(4));
		Consigne_red = _IQrsmpy(Consigne_red_temp, _IQ29(0.04));
//		Consigne_red = Consigne_red>>3 + _IQ29(1);
		Consigne_red_l2 = _IQ22mpyIQX(_IQ22(76.4), 22, Consigne_red, 29) ;
		position1_return = _IQ22mpyIQX(_IQ22(76.4), 22, position1, 29);
		Consigne_red_l2 -= position1_return;
		speed1_return = _IQ22mpyIQX(_IQ22(20.6028), 22, speed1, 29);
		Consigne_red_l2 -= speed1_return;
		pre_int_return = position1 - Consigne_red;
		int_return += _IQrsmpy(_IQ29(0.01), pre_int_return);		
		int_return_l3 = _IQ22mpyIQX(_IQ22(3.16228),22,int_return,29);//- _IQ22mpy(cp, _IQ22(6)); //G*M*L/18
		vartemp22_1 = Consigne_red_l2 - int_return_l3;
		if (vartemp22_1 > _IQ22(1))
			vartemp22_1 = _IQ22(1);
		if (vartemp22_1 < _IQ22(-1))
			vartemp22_1 = _IQ22(-1);
		cmd_i0 =  _IQ22toIQ(vartemp22_1);

		// BO
		//cmd_i0 = ConsigneExt1;
		}
	else
		cmd_i0 = _IQ29(0);

	//limitation du couple à 25% de la valeur nominale
	if (cmd_i0 > _IQ29(0.25)) 
		cmd_i0 = _IQ29(0.25);
	else if (cmd_i0 < _IQ29(-0.25))
		cmd_i0 = _IQ29(-0.25);

	if (virtual) {
		/*	MCC model	*/
		//speed0 = (cmd_i1 * (1 - exp(-MU * DELTA / TM)) / MU) + (speed1 * exp(-MU * DELTA / TM));
		speed0 = _IQmpy(cmd_i1, _IQ29(0.0138)) + _IQmpy(speed1, _IQ29(0.986)); 
		//position0 = (DELTA * speed1) + position1;
		position0 = _IQmpy(speed1, _IQ29(0.01)) + position1;
		
		//increment indexes for next loop 
		speed1 = speed0;
		position1 = position0;
		cmd_i1 = cmd_i0;
	}
}
/**********************************************************************************/
/**********************************************************************************/




// Consigne de couple (Ext1) et de flux (Potar)
	Cref[NB_C-1] = cmd_i0;
	// Divise la consigne par 4 et sature en negatif
	ConsignePot = _IQsat(ConsignePot, _IQ29(1), _IQ29(0)) >> 2;
	PsiRef[NB_PSI-1] = ConsignePot;
	
// La consigne de couple est en fait une consigne de courant en quad.	   
	IqRef[NB_IDQ-1] = Cref[NB_C-1];
	
// Transformation de Park
	Park(Ia, Ib, Ic, &Id[NB_IDQ-1], &Iq[NB_IDQ-1], TetaPk);

// Estimation du Flux
	EstimFlux(&PsiR[NB_PSI-1],&Id[NB_IDQ-1]);
	 
// Boucle de correction du flux tous les NB_CYLCES_FLUX (12,5ms)
	BoucleFlux = BoucleFlux - 1;
	if (BoucleFlux == 0)
	{
//		if (PsiRef[NB_PSI-1] > (PsiRef[NB_PSI-2]+_IQ29(0.1)))
//			PsiRef[NB_PSI-1] = PsiRef[NB_PSI-2] + _IQ29(0.005);
		for (i=1; i<NB_PSI; i++)
		{
			PsiErr[i-1] = PsiErr[i];
		}
		for (i=1; i<NB_IDQ; i++)
		{
			IdRef[i-1] = IdRef[i];
		}
		BoucleFlux = NB_CYCLES_FLUX;
		// Calculs de l'erreur
		PsiErr[NB_PSI-1] = PsiRef[NB_PSI-1] - PsiR[NB_PSI-1];
		PsiErr[NB_PSI-1] = _IQsat(PsiErr[NB_PSI-1], _IQ29(1), _IQ29(-1));
		// Calculs de la correction
		CorrectFlux(&PsiErr[NB_PSI-1], &IdRef[NB_IDQ-1]);
		IdRef[NB_IDQ-1] = _IQsat(IdRef[NB_IDQ-1], _IQ29(1), _IQ29(-1));

		for (i=1; i<NB_PSI; i++)
		{
			PsiRef[i-1] = PsiRef[i];
		}
	}

// Boucle de correction du courant (62,5us)
	IdErr[NB_IDQ-1] = IdRef[NB_IDQ-1] - Id[NB_IDQ-1];
	CorrectCourant(&IdErr[NB_IDQ-1], &Vd[NB_VDQ-1]);

	IqErr[NB_IDQ-1] = IqRef[NB_IDQ-1] - Iq[NB_IDQ-1];
	CorrectCourant(&IqErr[NB_IDQ-1], &Vq[NB_VDQ-1]);

// Estimation du repère de Park
	OrientationPark(WR, Iq[NB_IDQ-1], PsiR[NB_PSI-1], &TetaPk, &WPk);

// Transformation de Park inverse
	ParkInv(Vd[NB_VDQ-1],Vq[NB_VDQ-1], &Vref_a, &Vref_b, &Vref_c, TetaPk);

	Commande = ConsigneExt1;

/*
// ConsignePot = ConsigneExt2;
	//ConsignePot = ConsignePot;
// µµµµµµµµµµµµµµµµµµµµ Scalaire
	Vcrete = _IQ29mpy(_IQ29(2), ConsignePot);
// Fabrication d'un angle.
// Inhibition du mode de gestion de l'overflow
	CLROVM;
	ThetaPk = ThetaPk + _IQ29mpy(ConsignePot, _IQ29(0.0130));
// Activation du mode de gestion de l'overflow
	SETOVM;
// 3 phases, pas de déphasage, angle fourni varie entre -4 et +4, Machine à 1 paire de pôles
	RefSinus(3, ThetaPk, 1, Vcrete, _IQ29(0), &Vref_a, &Vref_b, &Vref_c);
	Commande = ConsignePot;
// µµµµµµµµµµµµµµµµµµµµ Fin Scalaire
*/


/*****************************************************************
 Depart et securité
 ****************************************************************/	
 /* Verification des grandeurs captées. Inhibition des PWM si
 le courant côté alternatif dépasse une valeur prohibitive 
 ou le bus continu dépasse une valeur prohibitive.
 Ces constantes sont définies dans Constante.h par Imax et UOmax */
	if ( ControleGrandAna(Ia, Ib, Ic, U0) == 0 )
	{
		Defaut = 1;
		DepartArretPWM = 0;
	}
	

// Inhibition des sorties de la PWM et du bit de frein
#if TargetCard == 1
// ***************** VERSION CARTE COMMANDE  *************************
//	DepartArretPWM = PupitreMarcheArret;
// Démarrage par au passage par 0
//	Defaut = PupitreModeCde;			// Juste pour test


	if ( ConsignePot > _IQ29(0.1) )
	{
		LedVisual1 = 1;		// Led droite allumée
	}
	else
	{
		LedVisual1 = 0;		// Led droite éteinte
	}
	
	if ( _IQ29abs(ConsigneExt1) < _IQ29(0.001) )
	{
		LedVisual2 = 1;		// Led gauche allumée
	}
	else
	{
		LedVisual2 = 0;		// Led gauche éteinte
	}
//
// Si on met en marche	
	if ( PupitreMarcheArret == 1 )
	{
// S'il n'y a pas de défaut ET que la PWM était inactive, alors on ne peut démarrer qu'au passage pas 0	
		if ( (Defaut == 0) && (DepartArretPWM == 0) )
		{
			if ( (LedVisual1 == 1) && (LedVisual2 == 1) )
				DepartArretPWM = 1;
			else
				DepartArretPWM = 0;
		}
	}
	else
	{
		DepartArretPWM = 0;
		Defaut = 0;
	}
	Commande_prec = Commande;
#else
if ( PupitreMarcheArret == 1 ) 
	DepartArretPWM = 1;
else
	DepartArretPWM = 0;
#endif 


/*****************************************************************
 Activation des PWM
 ****************************************************************/
	ActivTempsMorts(SET_DEAD_TIME);


// Inhibition des sorties de la PWM et du bit de frein
#if TargetCard == 0
// ***************** VERSION CARTE DEVELOPPEMENT  ********************
	ActivPWM(DepartArretPWM, 0, 0);
#else
	ActivPWM(DepartArretPWM, 0);
#endif

	
/*****************************************************************
 Envoi des commandes
 ****************************************************************/
// Initialisation du compteur
	SETOVM;
	if ( (DepartArretPWM) == 0 ) {
		magnet = 1;
		load = 0;
		compteur_demarrage = INIT_COMPTEUR;
	}
// Par défaut dans constante.h, le modulateur optimisé est activé
// et le controle du démarrage est désactivé	
	else
	{
// Si le controle progressif du démmarrrage est activé. On sature pendant 2 secondes
		if (START_MODE == 1)
		{
			if (compteur_demarrage > 0)
				compteur_demarrage = compteur_demarrage - 1;
	
// Fenetre ouvrant le rapport cyclique pendant le démarrage
			if (compteur_demarrage > 0)
			{
				LimiteV = _IQ29(1) - _IQ29(compteur_demarrage * UnSurInitCompteur_Def);
				Vref_a = _IQsat(Vref_a, LimiteV, -LimiteV);
				Vref_b = _IQsat(Vref_b, LimiteV, -LimiteV);
				Vref_c = _IQsat(Vref_c, LimiteV, -LimiteV);
			}
			else
				magnet = 0;
		}
	
/*****************************************************************
 Envoi des commandes
 ****************************************************************/
		Kmod = _IQ29(0.5);

// Modulation optimisee
		if (SET_MOD_OPT == 1)
		{
			Vref_a_Opt = Vref_a;
			Vref_b_Opt = Vref_b;
			Vref_c_Opt = Vref_c;
			PWMOptim(&Vref_a_Opt, &Vref_b_Opt, &Vref_c_Opt);

			SetPWM(Vref_a_Opt, Kmod, 1, _IQ29(DutyMin), _IQ29(DutyMax));
			SetPWM(Vref_b_Opt, Kmod, 2, _IQ29(DutyMin), _IQ29(DutyMax));
			SetPWM(Vref_c_Opt, Kmod, 3, _IQ29(DutyMin), _IQ29(DutyMax));
		}
		else
		{
// Modulateur normal
			SetPWM(Vref_a, Kmod, 1, _IQ29(DutyMin), _IQ29(DutyMax));
			SetPWM(Vref_b, Kmod, 2, _IQ29(DutyMin), _IQ29(DutyMax));
			SetPWM(Vref_c, Kmod, 3, _IQ29(DutyMin), _IQ29(DutyMax));
		}
	}



/*****************************************************************
 On renvoie l'entrée 1 sur la sortie 1 (CAN1 --> CNA1)	et l'entrée
 2 sur la sortie 2 (CAN2 --> CNA2)	
 *****************************************************************/
#if TargetCard == 1
	SortieCNA1 = ModeCdeBit0;
#endif
	switch (SortieCNA1)
	{
		case 0:
	  		AnalogOut(position1, 1);	
			break;
		case 1:	 
			AnalogOut(ThetaR[9], 1);
			break;
		case 2:
			AnalogOut(speed1, 1);
			break;
		case 3:
			AnalogOut(Ib, 1);
	}

#if TargetCard == 1
	SortieCNA2 = ModeCdeBit1;
#endif
	switch (SortieCNA2)
	{
		case 0:
	  		AnalogOut(cmd_i0, 2);
			break;
		case 1:
			AnalogOut(WR, 2);
			break;
		case 2:
			AnalogOut(Cref[NB_C-1], 2);
		case 3:
	  		AnalogOut(ConsigneExt1, 2);
	}

	
/*********************************** A MODIFIER POUR CARTE FINALE  **************************/
/* Sur la carte finale 2 sortie analogiques supplémentaires sont disponibles. Pour les utiliser, décommenter les
 ligne suivantes, */
/**********************************************************************************************/
#if TargetCard == 1
// ***************** VERSION CARTE COMMANDE  *************************
	SortieCNA3 = ModeCdeBit2;
	switch (SortieCNA3)
	{
		case 0:
			AnalogOut(WR, 3);
			break;
		case 1:
	  		AnalogOut(Id[NB_IDQ-1], 3);	
	}

	SortieCNA4 = ModeCdeBit3;
	switch (SortieCNA4)
	{
		case 0:
	  		AnalogOut(ThetaR[1], 4);
			break;
		case 1:
			AnalogOut(Iq[NB_IDQ-1], 4);
	}
#endif
	
	TogCalcTime = 1;

}

//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!



