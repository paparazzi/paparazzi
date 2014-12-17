/** Automatic survey of an oriented rectangle (defined by three points)  */

#ifndef CARTOGRAPHY_H
#define CARTOGRAPHY_H




void init_carto(void);
void periodic_downlink_carto(void);
void start_carto(void);
void stop_carto(void);

/*
 typedef enum {NS, WE} survey_orientation_t;
 */


#if USE_ONBOARD_CAMERA
extern bool_t CAMERA_SNAPSHOT_REQUIERED;
extern uint16_t camera_snapshot_image_number;
#endif

extern float distrailinteractif; //pour exporter la variable et pouvoir la changer depuis settings



///////////////////////////////////////////////////////////////////////////////////////////////


extern bool_t nav_survey_Inc_railnumberSinceBoot(void);
extern bool_t nav_survey_Snapshoot(void);
bool_t nav_survey_Snapshoot_Continu(void);
extern bool_t nav_survey_StopSnapshoot(void);
extern bool_t nav_survey_computefourth_corner(uint8_t wp1, uint8_t wp2,  uint8_t wp3, uint8_t wp4);

extern bool_t nav_survey_losange_carto_init(uint8_t wp1, uint8_t wp2,  uint8_t wp3, float distrail, float distplus);

extern bool_t nav_survey_losange_carto(
  void);   // !!!! important il faut mettre void en parametres d'entrée, sinon le compilo dit: attention : function declaration isn»t a prototype

//(uint8_t wp1, uint8_t wp2, uint8_t wp3);

/*
 #define NavSurveylosange_cartoInit(_wp1, _wp2, _grid, _distplus) nav_survey_losange_init(_wp1, _wp2, _wp3, _grid, _distplus)
 #define NavSurveylosange_carto nav_survey_losange
 */



#endif

