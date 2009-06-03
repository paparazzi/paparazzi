/*
 * $Id$
 *  
 * Copyright (C) 2008 Gautier Hattenberger
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA. 
 *
 */

#include "sim_ac_jsbsim.h"
#include "main_ap.h"
#include "main_fbw.h"

using namespace JSBSim;

//static void sim_gps_feed_data(void);
//static void sim_ir_feed_data(void);

void autopilot_init(void) {
  init_fbw();
  init_ap();
}

void autopilot_periodic_task(void) {
  periodic_task_ap();
  periodic_task_fbw();
}

void autopilot_event_task(void) {
  event_task_ap();
  event_task_fbw();
}


void copy_inputs_to_jsbsim(JSBSim::FGFDMExec* FDMExec) {
  
  FGPropagate* Propagate;

  Propagate = FDMExec->GetPropagate();
  
  cout << "Input " << Propagate->GetEuler(FGJSBBase::ePhi) << endl;
  cout << "Input " << Propagate->GetEuler(FGJSBBase::eTht) << endl;
  cout << "Input " << Propagate->GetEuler(FGJSBBase::ePsi) << endl;

}

void copy_outputs_from_jsbsim(JSBSim::FGFDMExec* FDMExec) {
  
  
  FGPropagate* Propagate;  

  Propagate = FDMExec->GetPropagate();
  
  cout << "Output " << Propagate->GetEuler(FGJSBBase::ePhi) << endl;
  cout << "Output " << Propagate->GetEuler(FGJSBBase::eTht) << endl;
  cout << "Output " << Propagate->GetEuler(FGJSBBase::ePsi) << endl;
  
}

/*
// Convert from the JSBsim generic_ struct to the FGInterface struct

bool FGJSBsim::copy_from_JSBsim()
{
    unsigned int i, j;
    
    _set_Inertias( MassBalance->GetMass(),
                   MassBalance->GetIxx(),
                   MassBalance->GetIyy(),
                   MassBalance->GetIzz(),
                   MassBalance->GetIxz() );
  
    _set_CG_Position( MassBalance->GetXYZcg(1),
                      MassBalance->GetXYZcg(2),
                      MassBalance->GetXYZcg(3) );

    _set_Accels_Body( Aircraft->GetBodyAccel(1),
                      Aircraft->GetBodyAccel(2),
                      Aircraft->GetBodyAccel(3) );

    _set_Accels_CG_Body_N ( Aircraft->GetNcg(1),
                            Aircraft->GetNcg(2),
                            Aircraft->GetNcg(3) );

    _set_Accels_Pilot_Body( Auxiliary->GetPilotAccel(1),
                            Auxiliary->GetPilotAccel(2),
                            Auxiliary->GetPilotAccel(3) );

    _set_Nlf( Aircraft->GetNlf() );

    // Velocities

    _set_Velocities_Local( Propagate->GetVel(FGJSBBase::eNorth),
                           Propagate->GetVel(FGJSBBase::eEast),
                           Propagate->GetVel(FGJSBBase::eDown) );

    _set_Velocities_Wind_Body( Propagate->GetUVW(1),
                               Propagate->GetUVW(2),
                               Propagate->GetUVW(3) );

    // Make the HUD work ...
    _set_Velocities_Ground( Propagate->GetVel(FGJSBBase::eNorth),
                            Propagate->GetVel(FGJSBBase::eEast),
                            -Propagate->GetVel(FGJSBBase::eDown) );

    _set_V_rel_wind( Auxiliary->GetVt() );

    _set_V_equiv_kts( Auxiliary->GetVequivalentKTS() );

    _set_V_calibrated_kts( Auxiliary->GetVcalibratedKTS() );

    _set_V_ground_speed( Auxiliary->GetVground() );

    _set_Omega_Body( Propagate->GetPQR(FGJSBBase::eP),
                     Propagate->GetPQR(FGJSBBase::eQ),
                     Propagate->GetPQR(FGJSBBase::eR) );

    _set_Euler_Rates( Auxiliary->GetEulerRates(FGJSBBase::ePhi),
                      Auxiliary->GetEulerRates(FGJSBBase::eTht),
                      Auxiliary->GetEulerRates(FGJSBBase::ePsi) );

    _set_Mach_number( Auxiliary->GetMach() );

    // Positions of Visual Reference Point
    FGLocation l = Auxiliary->GetLocationVRP();
    _updateGeocentricPosition( l.GetLatitude(), l.GetLongitude(),
                               l.GetRadius() - get_Sea_level_radius() );

    _set_Altitude_AGL( Propagate->GetDistanceAGL() );
    {
      double loc_cart[3] = { l(FGJSBBase::eX), l(FGJSBBase::eY), l(FGJSBBase::eZ) };
      double contact[3], d[3], sd, t;
      is_valid_m(&t, d, &sd);
      get_agl_ft(t, loc_cart, SG_METER_TO_FEET*2, contact, d, d, &sd);
      double rwrad
        = FGColumnVector3( contact[0], contact[1], contact[2] ).Magnitude();
      _set_Runway_altitude( rwrad - get_Sea_level_radius() );
    }

    _set_Euler_Angles( Propagate->GetEuler(FGJSBBase::ePhi),
                       Propagate->GetEuler(FGJSBBase::eTht),
                       Propagate->GetEuler(FGJSBBase::ePsi) );

    _set_Alpha( Auxiliary->Getalpha() );
    _set_Beta( Auxiliary->Getbeta() );


    _set_Gamma_vert_rad( Auxiliary->GetGamma() );

    _set_Earth_position_angle( Inertial->GetEarthPositionAngle() );

    _set_Climb_Rate( Propagate->Gethdot() );

    const FGMatrix33& Tl2b = Propagate->GetTl2b();
    for ( i = 1; i <= 3; i++ ) {
        for ( j = 1; j <= 3; j++ ) {
            _set_T_Local_to_Body( i, j, Tl2b(i,j) );
        }
    }

    // Copy the engine values from JSBSim.
    for ( i=0; i < Propulsion->GetNumEngines(); i++ ) {
      SGPropertyNode * node = fgGetNode("engines/engine", i, true);
      char buf[30];
      sprintf(buf, "engines/engine[%d]/thruster", i);
      SGPropertyNode * tnode = fgGetNode(buf, true);
      FGThruster * thruster = Propulsion->GetEngine(i)->GetThruster();

      switch (Propulsion->GetEngine(i)->GetType()) {
      case FGEngine::etPiston:
        { // FGPiston code block
        FGPiston* eng = (FGPiston*)Propulsion->GetEngine(i);
        node->setDoubleValue("egt-degf", eng->getExhaustGasTemp_degF());
        node->setDoubleValue("oil-temperature-degf", eng->getOilTemp_degF());
        node->setDoubleValue("oil-pressure-psi", eng->getOilPressure_psi());
        node->setDoubleValue("mp-osi", eng->getManifoldPressure_inHg());
        // NOTE: mp-osi is not in ounces per square inch.
        // This error is left for reasons of backwards compatibility with
        // existing FlightGear sound and instrument configurations.
        node->setDoubleValue("mp-inhg", eng->getManifoldPressure_inHg());
        node->setDoubleValue("cht-degf", eng->getCylinderHeadTemp_degF());
        node->setDoubleValue("rpm", eng->getRPM());
        } // end FGPiston code block
        break;
      case FGEngine::etRocket:
        { // FGRocket code block
        FGRocket* eng = (FGRocket*)Propulsion->GetEngine(i);
        } // end FGRocket code block
        break;
      case FGEngine::etTurbine:
        { // FGTurbine code block
        FGTurbine* eng = (FGTurbine*)Propulsion->GetEngine(i);
        node->setDoubleValue("n1", eng->GetN1());
        node->setDoubleValue("n2", eng->GetN2());
        node->setDoubleValue("egt_degf", 32 + eng->GetEGT()*9/5);
        node->setBoolValue("augmentation", eng->GetAugmentation());
        node->setBoolValue("water-injection", eng->GetInjection());
        node->setBoolValue("ignition", eng->GetIgnition());
        node->setDoubleValue("nozzle-pos-norm", eng->GetNozzle());
        node->setDoubleValue("inlet-pos-norm", eng->GetInlet());
        node->setDoubleValue("oil-pressure-psi", eng->getOilPressure_psi());
        node->setBoolValue("reversed", eng->GetReversed());
        node->setBoolValue("cutoff", eng->GetCutoff());
        node->setDoubleValue("epr", eng->GetEPR());
        globals->get_controls()->set_reverser(i, eng->GetReversed() );
        globals->get_controls()->set_cutoff(i, eng->GetCutoff() );
        globals->get_controls()->set_water_injection(i, eng->GetInjection() );
        globals->get_controls()->set_augmentation(i, eng->GetAugmentation() );
        } // end FGTurbine code block
        break;
      case FGEngine::etTurboprop:
        { // FGTurboProp code block
        FGTurboProp* eng = (FGTurboProp*)Propulsion->GetEngine(i);
        node->setDoubleValue("n1", eng->GetN1());
        //node->setDoubleValue("n2", eng->GetN2());
        node->setDoubleValue("itt_degf", 32 + eng->GetITT()*9/5);
        node->setBoolValue("ignition", eng->GetIgnition());
        node->setDoubleValue("nozzle-pos-norm", eng->GetNozzle());
        node->setDoubleValue("inlet-pos-norm", eng->GetInlet());
        node->setDoubleValue("oil-pressure-psi", eng->getOilPressure_psi());
        node->setBoolValue("reversed", eng->GetReversed());
        node->setBoolValue("cutoff", eng->GetCutoff());
        node->setBoolValue("starting", eng->GetEngStarting());
        node->setBoolValue("generator-power", eng->GetGeneratorPower());
        node->setBoolValue("damaged", eng->GetCondition());
        node->setBoolValue("ielu-intervent", eng->GetIeluIntervent());
        node->setDoubleValue("oil-temperature-degf", eng->getOilTemp_degF());
//        node->setBoolValue("onfire", eng->GetFire());
        globals->get_controls()->set_reverser(i, eng->GetReversed() );
        globals->get_controls()->set_cutoff(i, eng->GetCutoff() );
        } // end FGTurboProp code block
        break;
      case FGEngine::etElectric:
        { // FGElectric code block
        FGElectric* eng = (FGElectric*)Propulsion->GetEngine(i);
        node->setDoubleValue("rpm", eng->getRPM());
        } // end FGElectric code block
        break;
      }

      { // FGEngine code block
      FGEngine* eng = Propulsion->GetEngine(i);
      node->setDoubleValue("fuel-flow-gph", eng->getFuelFlow_gph());
      node->setDoubleValue("thrust_lb", thruster->GetThrust());
      node->setDoubleValue("fuel-flow_pph", eng->getFuelFlow_pph());
      node->setBoolValue("running", eng->GetRunning());
      node->setBoolValue("starter", eng->GetStarter());
      node->setBoolValue("cranking", eng->GetCranking());
      globals->get_controls()->set_starter(i, eng->GetStarter() );
      } // end FGEngine code block

      switch (thruster->GetType()) {
      case FGThruster::ttNozzle:
        { // FGNozzle code block
        FGNozzle* noz = (FGNozzle*)thruster;
        } // end FGNozzle code block
        break;
      case FGThruster::ttPropeller:
        { // FGPropeller code block
        FGPropeller* prop = (FGPropeller*)thruster;
        tnode->setDoubleValue("rpm", thruster->GetRPM());
        tnode->setDoubleValue("pitch", prop->GetPitch());
        tnode->setDoubleValue("torque", prop->GetTorque());
        tnode->setBoolValue("feathered", prop->GetFeather());
        } // end FGPropeller code block
        break;
      case FGThruster::ttRotor:
        { // FGRotor code block
        FGRotor* rotor = (FGRotor*)thruster;
        } // end FGRotor code block
        break;
      case FGThruster::ttDirect:
        { // Direct code block
        } // end Direct code block
        break;
      }

    }

    // Copy the fuel levels from JSBSim if fuel
    // freeze not enabled.
    if ( ! Propulsion->GetFuelFreeze() ) {
      for (i = 0; i < Propulsion->GetNumTanks(); i++) {
        SGPropertyNode * node = fgGetNode("/consumables/fuel/tank", i, true);
        FGTank* tank = Propulsion->GetTank(i);
        double contents = tank->GetContents();
        double temp = tank->GetTemperature_degC();
        node->setDoubleValue("level-gal_us", contents/6.6);
        node->setDoubleValue("level-lbs", contents);
        if (temp != -9999.0) node->setDoubleValue("temperature_degC", temp);
      }
    }

    update_gear();

    stall_warning->setDoubleValue( Aerodynamics->GetStallWarn() );

    elevator_pos_pct->setDoubleValue( FCS->GetDePos(ofNorm) );
    left_aileron_pos_pct->setDoubleValue( FCS->GetDaLPos(ofNorm) );
    right_aileron_pos_pct->setDoubleValue( FCS->GetDaRPos(ofNorm) );
    rudder_pos_pct->setDoubleValue( -1*FCS->GetDrPos(ofNorm) );
    flap_pos_pct->setDoubleValue( FCS->GetDfPos(ofNorm) );
    speedbrake_pos_pct->setDoubleValue( FCS->GetDsbPos(ofNorm) );
    spoilers_pos_pct->setDoubleValue( FCS->GetDspPos(ofNorm) );
    tailhook_pos_pct->setDoubleValue( FCS->GetTailhookPos() );
    wing_fold_pos_pct->setDoubleValue( FCS->GetWingFoldPos() );

    // force a sim crashed if crashed (altitude AGL < 0)
    if (get_Altitude_AGL() < -100.0) {
         State->SuspendIntegration();
         crashed = true;
    }

    return true;
    }

 */

//#include "gps.h"
//static void sim_gps_feed_data(void) {
//}

