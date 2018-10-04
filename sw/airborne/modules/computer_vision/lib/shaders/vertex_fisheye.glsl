attribute vec4 aVertexPosition;
attribute vec4 inputTextureCoordinate;

varying vec4 vPosition;
varying vec2 vTextureCoord;

void main() {
	vPosition = aVertexPosition;
	vTextureCoord = inputTextureCoordinate.xy;
	gl_Position = vPosition;
}
