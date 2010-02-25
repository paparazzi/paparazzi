//POVRay include files
#include "arrays.inc"
#include "chars.inc"
#include "colors.inc"
#include "finish.inc"
#include "glass.inc"
#include "golds.inc"
#include "math.inc"
#include "metals.inc"
#include "rand.inc"
#include "shapes.inc"
#include "shapes2.inc"
#include "shapesq.inc"
#include "skies.inc"
#include "stoneold.inc"
#include "stones.inc"
#include "stones1.inc"
#include "stones2.inc"
#include "strings.inc"
#include "textures.inc"
#include "transforms.inc"

#macro AXIS_POVRAY(mac_x_ver,mac_y_ver,mac_z_ver,mac_x_rot,mac_y_rot,mac_z_rot)
object {
  union {
    union {
      cylinder{<0.,0.,0.><2500.,0.,0.> 10}
      text { ttf "timrom.ttf" "X" 1, 0 scale <2,2,2> translate(<20, 0, 0>)}
      texture{ pigment{rgb<1,0,0>}}
    }   
    union {
      cylinder{<0.,0,0.><0.,2500.,0.> 10}
      text { ttf "timrom.ttf" "Y" 1, 0 scale <2,2,2> translate(<0, 20, 0>)}
      texture{ pigment{rgb<0,1,0>}}
    }   
    union {
      cylinder{<0.,0,0.><0.,0.,2500.> 10}
      text { ttf "timrom.ttf" "Z" 1, 0 scale <2,2,2> translate(<0, 0, 20>)}
      texture{ pigment{rgb<0,0,1>}}
    }   
  translate(<mac_x_ver,mac_y_ver,mac_z_ver>)
  rotate(<mac_x_rot,mac_y_rot,mac_z_rot>)	
  }
} 
#end


#macro AXIS_NED()
object {
  union {
    union {
      cylinder{<0.,0.,0.><2500.,0.,0.> 10}
      texture{ pigment{rgb<1,0,0>}}
    }   
    union {
      cylinder{<0.,0,0.><0.,2500.,0.> 10}
      texture{ pigment{rgb<0,1,0>}}
    }   
    union {
      cylinder{<0.,0,0.><0.,0.,2500.> 10}
      texture{ pigment{rgb<0,0,1>}}
    }   
  }
  matrix < 1, 0, 0,
           0, 0,-1,
           0,-1, 0,
           0, 0, 0 >
} 
#end



#macro Q6D(xned, yned, zned, phi, theta, psi)

object {
  union {
    box{<-250,-10,-10><0,10,10> texture{pigment{Blue}}}
    box{<0,-10,-10><250,10,10> texture{pigment{Red}}}
    cylinder{<-250,0,-10><-250,0,-15>  125 texture{pigment{Green}}}
    cylinder{<250, 0,-10><250,0,-15>   125 texture{pigment{Green}}}

    box{<-10,-250,-10><10,250,10> texture{pigment{Blue}}}
    cylinder{<0,-250,-10><0,-250,-15>  125 texture{pigment{Green}}}
    cylinder{<0, 250,-10><0, 250,-15>  125 texture{pigment{Green}}}
  }
  rotate <phi,theta, psi>
  translate <xned,yned,zned>
  matrix < 1, 0, 0,
           0, 0,-1,
           0,-1, 0,
           0, 0, 0 >
}
#end


#declare cam_x = 0;
#declare cam_y = 5000;
#declare cam_z = 20000;

#declare cam_look_x = 0;
#declare cam_look_y = 0;
#declare cam_look_z = 0;

#declare cam_a = 36;
//#declare cam_a = 10;

camera
{
	location <cam_x,cam_y,cam_z>
	look_at <cam_look_x,cam_look_y,cam_look_z>
	angle cam_a
}

#local lgt1_pos_x = 3400;
#local lgt1_pos_y = 5100;
#local lgt1_pos_z = 2600;
#local lgt1_intense = 0.763488;
#local lgt2_pos_x = -3400;
#local lgt2_pos_y = 5100;
#local lgt2_pos_z = 2600;
#local lgt2_intense = 0.763488;
#local lgt3_pos_x = 3400;
#local lgt3_pos_y = 5100;
#local lgt3_pos_z = -1700;
#local lgt3_intense = 0.763488;
#local lgt4_pos_x = -3400;
#local lgt4_pos_y = 5100;
#local lgt4_pos_z = -1700;
#local lgt4_intense = 0.763488;

light_source{<lgt1_pos_x,lgt1_pos_y,lgt1_pos_z> White*lgt1_intense}
light_source{<lgt2_pos_x,lgt2_pos_y,lgt2_pos_z> White*lgt2_intense}
light_source{<lgt3_pos_x,lgt3_pos_y,lgt3_pos_z> White*lgt3_intense}
light_source{<lgt4_pos_x,lgt4_pos_y,lgt4_pos_z> White*lgt4_intense}


//plane { -y, 3000 texture{T_Chrome_2D normal{waves 0.1 frequency 3000.0 scale 30.0}} translate<0,0,0>}
//plane { -z, 3000 texture{T_Chrome_2D normal{waves 0.1 frequency 3000.0 scale 30.0}} translate<0,0,0>}

plane { -y, 3000 texture{T_Chrome_2D}}
//plane { -z, 3000 texture{T_Chrome_2D}}

sky_sphere {pigment {Navy}
pigment {bozo turbulence 0.65 octaves 7 omega 0.7 lambda 2
color_map {
[0.0 0.1 color rgb <0.85, 0.85, 0.85> color rgb <0.75, 0.75, 0.75>]
[0.1 0.5 color rgb <0.75, 0.75, 0.75> color rgbt <1, 1, 1, 1>]
[0.5 1.0 color rgbt <1, 1, 1, 1> color rgbt <1, 1, 1, 1>]}
scale <0.1, 0.5, 0.1>} rotate 90*z}

background{Gray50}