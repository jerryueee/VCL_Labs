#version 410 core

layout(location = 0) in  vec3 v_Position;
layout(location = 1) in  vec3 v_Normal;
layout(location = 2) in  vec2 v_TexCoord;

layout(location = 0) out vec4 f_Color;

struct Light {
    vec3  Intensity;
    vec3  Direction;   // For spot and directional lights.
    vec3  Position;    // For point and spot lights.
    float CutOff;      // For spot lights.
    float OuterCutOff; // For spot lights.
    // 内外圆锥角

};

// 从应用程序传入着色器的一组全局常量
layout(std140) uniform PassConstants {
    mat4  u_Projection;//投影矩阵，用于将场景投影到屏幕坐标
    mat4  u_View;//视图矩阵，模拟相机的视角
    vec3  u_ViewPosition;// 观察者位置，用于计算光照等
    vec3  u_AmbientIntensity;//环境光强度
    Light u_Lights[4];
    int   u_CntPointLights;
    int   u_CntSpotLights;
    int   u_CntDirectionalLights;
};

uniform float u_AmbientScale;
uniform bool  u_UseBlinn;
uniform float u_Shininess;
uniform bool  u_UseGammaCorrection;
uniform int   u_AttenuationOrder;
uniform float u_BumpMappingBlend;

//sampler2D指定了一个二维纹理采样器
uniform sampler2D u_DiffuseMap;
uniform sampler2D u_SpecularMap;
uniform sampler2D u_HeightMap;

vec3 Shade(vec3 lightIntensity, vec3 lightDir, vec3 normal, vec3 viewDir, vec3 diffuseColor, vec3 specularColor, float shininess) {
    // your code here:
    vec3 L;
    if (u_UseBlinn){
        vec3 h = normalize(lightDir + viewDir);
        L = diffuseColor * lightIntensity * max(0, dot(normal, lightDir)) + specularColor * lightIntensity * pow(max(0, dot(normal, h)), shininess);
    }
    else {
        vec3 r = -lightDir + 2 * dot(lightDir, normal) * normal;
        L = diffuseColor * lightIntensity * max(0, dot(normal, lightDir)) + specularColor * lightIntensity * pow(max(0, dot(r, viewDir)), shininess);
    }
    return L;
}

vec3 GetNormal() {
    // Bump mapping from paper: Bump Mapping Unparametrized Surfaces on the GPU
    vec3 vn = normalize(v_Normal);
    
    // your code here:
    vec3 dpx = dFdx(v_Position.xyz);
    vec3 dpy = dFdy(v_Position.xyz);
    
    vec3 r1 = cross(dpy, vn);
    vec3 r2 = cross(vn, dpx);//r1，r2代表表面在x和y方向的切线
    float det = dot(dpx, r1);
    float Hll = texture(u_HeightMap, v_TexCoord).x;
    float Hlr = texture(u_HeightMap, v_TexCoord + dFdx(v_TexCoord.xy)).x;//x方向偏移的高度值
    float Hul = texture(u_HeightMap, v_TexCoord + dFdy(v_TexCoord.xy)).x;//y方向偏移的高度值

    vec3 surf_grad = sign(det) * ((Hlr - Hll) * r1 + (Hul - Hll) * r2);//注意正负方向
    vec3 bumpNormal = vn * (1. - u_BumpMappingBlend) + normalize(abs(det) * vn - surf_grad) * u_BumpMappingBlend;
    return bumpNormal;
    //return bumpNormal != bumpNormal ? vn : normalize(vn * (1. - u_BumpMappingBlend) + bumpNormal * u_BumpMappingBlend);
}

void main() {
    float gamma          = 2.2;
    vec4  diffuseFactor  = texture(u_DiffuseMap , v_TexCoord).rgba;
    vec4  specularFactor = texture(u_SpecularMap, v_TexCoord).rgba;
    if (diffuseFactor.a < .2) discard;//丢弃当前片段
    vec3  diffuseColor   = u_UseGammaCorrection ? pow(diffuseFactor.rgb, vec3(gamma)) : diffuseFactor.rgb;
    vec3  specularColor  = specularFactor.rgb;
    float shininess      = u_Shininess < 0 ? specularFactor.a * 256 : u_Shininess;
    vec3  normal         = GetNormal();
    vec3  viewDir        = normalize(u_ViewPosition - v_Position);// 视线
    // Ambient component.
    vec3  total = u_AmbientIntensity * u_AmbientScale * diffuseColor;
    // Iterate lights.
    for (int i = 0; i < u_CntPointLights; i++) {
        vec3  lightDir     = normalize(u_Lights[i].Position - v_Position);
        float dist         = length(u_Lights[i].Position - v_Position);
        float attenuation  = 1. / (u_AttenuationOrder == 2 ? dist * dist : (u_AttenuationOrder == 1  ? dist : 1.));//计算光照衰减因子，shade计算时先不考虑
        total             += Shade(u_Lights[i].Intensity, lightDir, normal, viewDir, diffuseColor, specularColor, shininess) * attenuation;
    }//点光源
    for (int i = u_CntPointLights + u_CntSpotLights; i < u_CntPointLights + u_CntSpotLights + u_CntDirectionalLights; i++) {
        total += Shade(u_Lights[i].Intensity, u_Lights[i].Direction, normal, viewDir, diffuseColor, specularColor, shininess);
    }//聚光灯，环境光
    // Gamma correction.
    f_Color = vec4(u_UseGammaCorrection ? pow(total, vec3(1. / gamma)) : total, 1.);
}
