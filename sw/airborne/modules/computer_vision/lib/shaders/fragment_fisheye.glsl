precision highp float;
uniform sampler2D videoFrame;

uniform vec4 uLens;
uniform vec2 uFov;

varying vec4 vPosition;
varying vec2 vTextureCoord;

vec2 GLCoord2TextureCoord(vec2 glCoord) {
	return glCoord  * vec2(1.0, -1.0)/ 2.0 + vec2(0.5, 0.5);
}

void main(){
	float scale = uLens.w;
	float F = uLens.z;

	float L = length(vec3(vPosition.xy/scale, F));

	vec2 vMapping = vPosition.xy * F / L;
	vMapping = vMapping * uLens.xy;

	vMapping = GLCoord2TextureCoord(vMapping/scale);

	vec4 texture = texture2D(videoFrame, vTextureCoord);
	if(vMapping.x > 0.99 || vMapping.x < 0.01 || vMapping.y > 0.99 || vMapping.y < 0.01){
		texture = vec4(0.5, 0.0, 0.5, 0.0);
	}
	gl_FragColor = texture;
}
