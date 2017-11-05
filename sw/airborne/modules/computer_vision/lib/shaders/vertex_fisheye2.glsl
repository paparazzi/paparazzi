#version 120
uniform mat4 gxl3d_ModelViewProjectionMatrix;
void main()
{
  gl_Position = gxl3d_ModelViewProjectionMatrix * gl_Vertex;
}
