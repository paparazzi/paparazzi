// Demodulation du signal VOR BF

// Suppression de toutes les variables non protegees
clear();
// Suppression de toutes les fenetres graphiques
xdel(winsid());

getf('exportCoeff.sci');

// Les parametres du signal VOR
Fvor = 30;
F0 = 9960;
// Indice de modulation de frequence du 30 REF
n = 16;
// Excursion de frequence instantanee produite par le 30 REF
Df = n*Fvor;


// Le signal recu, format WAV 16 bits mono
[r,Fe,bits] = wavread('signal_VOR_BF_50_200dB.wav');
// [r,Fe,bits] = wavread('SousEch/signal_VOR_BF25_50_200dB.wav');
r = r*2^6;
Te = 1/Fe;
t = [0:length(r)-1]*Te;
Lt = length(t);

//-----------------------------------------------------------------//
// ETUDE DU PASSE-BAS SERVANT A ISOLER LE 3O VAR ET LE CONTINU

filtreVAR = iir(3,'lp','butt',[300/Fe 0],[0 0]);
[dBvar,Dvar] = dbphi(repfreq(filtreVAR,Fvor/Fe));
ssFiltreVAR = tf2ss(filtreVAR);
num = coeff(filtreVAR.num);
den = coeff(filtreVAR.den);
exportCoeff("VAR",num($:-1:1),den($:-1:1));
// exportCoeff("VAR",num,den);
//-----------------------------------------------------------------//

//-----------------------------------------------------------------//
// ETUDE DU PASSE-BANDE SERVANT A ISOLER LE 3O REF

// La fonction de transfer en z
fc1 = (F0-Df)/Fe*0.9; fc2 = (F0+Df)/Fe*1.1; 
filtreREF = iir(3,'bp','butt',[fc1 fc2],[0 0]);
[dBref,Dref] = dbphi(repfreq(filtreREF,-0.5,F0/Fe));
Dref = modulo(Dref($)/F0*Fvor,360);
num = coeff(filtreREF.num);
den = coeff(filtreREF.den);
exportCoeff("REF",num,den);
//-----------------------------------------------------------------//

//-----------------------------------------------------------------//
// SELECTION DU 30 VAR

signalVAR = flts(r,filtreVAR);
//-----------------------------------------------------------------//

//-----------------------------------------------------------------//
// SELECTION DU 30 REF

signalREF = flts(r,filtreREF);
//-----------------------------------------------------------------//

// La frequence de base de l'oscillateur local 30 REF
Fref = F0;
// La phase initiale de l'oscillateur local
phiREF = %pi;
// L'erreur de phase
eREF = 0;
// Le coefficient de reinjection de l'erreur de phase.
alphaREF = -1.2;

//-----------------------------------------------------------------//
// PASSE-BAS RII PLL 30 REF
pbREF = iir(3,'lp','butt',[3000/Fe 0],[0 0]);
[dBfm,Dfm] = dbphi(repfreq(pbREF,Fvor/Fe));
ssREF = tf2ss(pbREF);
num = coeff(pbREF.num);
den = coeff(pbREF.den);
exportCoeff("PBREF",num,den);
//-----------------------------------------------------------------//

// La frequence de base de l'oscillateur local 30 VAR
Fvar = Fvor;
phiVAR = %pi;
eVAR = 0;
// Le coefficient de reinjection de l'erreur de phase.
alphaVAR = -0.5;

// Le facteur de decimation du 30 VAR ET DU 30 REF DEMODULE (FM)
decim = 83*3;

//-----------------------------------------------------------------//
// PASSE-BAS RII PLL 30 VAR
pbVAR = iir(6,'lp','butt',[10/Fe*decim 0],[0 0]);
ssVAR = tf2ss(pbVAR);
num = coeff(pbVAR.num);
den = coeff(pbVAR.den);
exportCoeff("PBVAR",num,den);
//-----------------------------------------------------------------//

// La frequence de base de l'oscillateur local 30 REF DEMODULE
Ffm = Fvor;
phiFM = %pi;
eFM = 0;
alphaFM = -1;

//-----------------------------------------------------------------//
// PASSE-BAS RII PLL 30 REF DEMODULE (FM)
pbFM = iir(6,'lp','butt',[10/Fe*decim 0],[0 0]);
ssFM = tf2ss(pbFM);
num = coeff(pbFM.num);
den = coeff(pbFM.den);
exportCoeff("PBFM",num,den);
//-----------------------------------------------------------------//

// Fréquence d'affichage
affich = round(Fe/2);

compt = decim;
j = 1;

//-----------------------------------------------------------------//
// LA BOUCLE A VERROUILLAGE DE PHASE  ( tourne a FE )
for i=1:Lt;
    
    ti = t(i);
    
    // La phase de l'oscillateur local
    phiREF = phiREF - alphaREF*eREF;
    phaseREF = 2*%pi*Fref*ti + phiREF;
    // La porteuse locale
    loREF = sin(phaseREF);
    
    // La multiplication du signal recu par la porteuse locale
    yREF = signalREF(i)*loREF;

    // L'erreur de phase, filtrage du produit de la multiplication
    // par un passe-bas.
    if (i==1)
      [eREF,etatREF] = flts(yREF,ssREF);
    else
      [eREF,etatREF] = flts(yREF,ssREF,etatREF);
    end
    
    // Il faut filtrer le 30 REF avant de le décimer !
    if (i==1)
      [eREFdecim,etatREFdecim] = flts(eREF,ssFiltreVAR);
    else
      [eREFdecim,etatREFdecim] = flts(eREF,ssFiltreVAR,etatREFdecim);
    end

    if (modulo(compt,decim) == 0)
      phiVAR = phiVAR - alphaVAR*eVAR;
      phaseVAR = 2*%pi*Fvar*ti + phiVAR;    
      loVAR = -sin(phaseVAR);
      yVAR = signalVAR(i)*loVAR;
      if (j==1)
        [eVAR,etatVAR] = flts(yVAR,ssVAR);
      else
        [eVAR,etatVAR] = flts(yVAR,ssVAR,etatVAR);       
      end      
      
      phiFM = phiFM - alphaFM*eFM;    
      phaseFM = 2*%pi*Ffm*ti + phiFM;  
      loFM = -sin(phaseFM);    
      yFM = eREFdecim*loFM;
      if (j==1)
        [eFM,etatFM] = flts(yFM,ssFM);
      else
        [eFM,etatFM] = flts(yFM,ssFM,etatFM);
      end

      j = j+1;      
    end
    
    if (modulo(compt,affich) == 0)
      leVAR = pmodulo(phiVAR*180/%pi-Dvar,360);
      leREF = pmodulo(phiFM*180/%pi-Dfm-Dref-Dvar,360);
      leQDR = pmodulo((phiVAR-phiFM)*180/%pi+Dfm+Dref,360);
      mprintf("Phase variable %2.2f",leVAR);
      mprintf(" Phase reference %2.2f",leREF);
      mprintf(" QDR %2.2f",leQDR);disp("");
    end
      
    compt = compt+1;
        
end
//-----------------------------------------------------------------//

