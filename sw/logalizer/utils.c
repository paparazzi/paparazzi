#include "utils.h"

#include <netinet/in.h>

//#include <simgear/io/lowlevel.hxx> // endian tests
void htond (double *x)
{
  //   if ( sgIsLittleEndian() ) {
        int    *Double_Overlay;
        int     Holding_Buffer;

        Double_Overlay = (int *) x;
        Holding_Buffer = Double_Overlay [0];

        Double_Overlay [0] = htonl (Double_Overlay [1]);
        Double_Overlay [1] = htonl (Holding_Buffer);
	//    } else {
	//        return;
	//    }
}

// Float version
void htonf (float *x)
{
  //    if ( sgIsLittleEndian() ) {
        int    *Float_Overlay;
        int     Holding_Buffer;

        Float_Overlay = (int *) x;
        Holding_Buffer = Float_Overlay [0];

        Float_Overlay [0] = htonl (Holding_Buffer);
	//    } else {
	//        return;
	//    }
}
