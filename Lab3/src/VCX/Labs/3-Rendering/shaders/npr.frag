#version 410 core

layout(location = 0) in  vec3 v_Position;
layout(location = 1) in  vec3 v_Normal;

layout(location = 0) out vec4 f_Color;

struct Light {
    vec3  Intensity;
    vec3  Direction;   // For spot and directional lights.
    vec3  Position;    // For point and spot lights.
    float CutOff;      // For spot lights.
    float OuterCutOff; // For spot lights.
};

layout(std140) uniform PassConstants {
    mat4  u_Projection;
    mat4  u_View;
    vec3  u_ViewPosition;
    vec3  u_AmbientIntensity;
    Light u_Lights[4];
    int   u_CntPointLights;
    int   u_CntSpotLights;
    int   u_CntDirectionalLights;
};

uniform vec3 u_CoolColor;
uniform vec3 u_WarmColor;

const float ks[3] = float[](0.33, 0.66, 0.99);

float find_k(float k){
    float mindis = abs(k - ks[0]);
    float ret = ks[0];
    for (int i = 1; i < 3; i++){
        float dis = abs(k - ks[i]);
        if (dis < mindis){
            mindis = dis;
            ret = ks[i];
        }
    }
    return ret;
}

vec3 Shade (vec3 lightDir, vec3 normal) {
    // your code here:
    float k1 = (1 + dot(normalize(lightDir), normalize(normal))) / 2.0;
    k1 = find_k(k1);
    float k2 = 1.0 - k1;
    vec3 color = k2 * u_CoolColor + k1 * u_WarmColor;
    return color;
}

void main() {
    // your code here:
    float gamma = 2.2;
    vec3 total = Shade(u_Lights[0].Direction, v_Normal);
    f_Color = vec4(pow(total, vec3(1. / gamma)), 1.);
}
