#include "Labs/3-Rendering/tasks.h"

namespace VCX::Labs::Rendering {

    glm::vec4 GetTexture(Engine::Texture2D<Engine::Formats::RGBA8> const & texture, glm::vec2 const & uvCoord) {
        if (texture.GetSizeX() == 1 || texture.GetSizeY() == 1) return texture.At(0, 0);
        glm::vec2 uv      = glm::fract(uvCoord);
        uv.x              = uv.x * texture.GetSizeX() - .5f;
        uv.y              = uv.y * texture.GetSizeY() - .5f;
        std::size_t xmin  = std::size_t(glm::floor(uv.x) + texture.GetSizeX()) % texture.GetSizeX();
        std::size_t ymin  = std::size_t(glm::floor(uv.y) + texture.GetSizeY()) % texture.GetSizeY();
        std::size_t xmax  = (xmin + 1) % texture.GetSizeX();
        std::size_t ymax  = (ymin + 1) % texture.GetSizeY();
        float       xfrac = glm::fract(uv.x), yfrac = glm::fract(uv.y);
        return glm::mix(glm::mix(texture.At(xmin, ymin), texture.At(xmin, ymax), yfrac), glm::mix(texture.At(xmax, ymin), texture.At(xmax, ymax), yfrac), xfrac);
    }

    glm::vec4 GetAlbedo(Engine::Material const & material, glm::vec2 const & uvCoord) {
        glm::vec4 albedo       = GetTexture(material.Albedo, uvCoord);
        glm::vec3 diffuseColor = albedo;
        return glm::vec4(glm::pow(diffuseColor, glm::vec3(2.2)), albedo.w);
    }

    /******************* 1. Ray-triangle intersection *****************/
    bool IntersectTriangle(Intersection & output, Ray const & ray, glm::vec3 const & p1, glm::vec3 const & p2, glm::vec3 const & p3) {
        // your code here
        glm::vec3 E1 = p2 - p1, E2  = p3 - p1, T = ray.Origin - p1, D = ray.Direction;
        glm::vec3 P = glm::cross(D, E2), Q = glm::cross(T, E1);
        float ratio = 1.0f / glm::dot(P, E1);
        float t = ratio * glm::dot(Q, E2);
        float u = ratio * glm::dot(P, T);
        float v = ratio * glm::dot(Q, D);
        if (t <= 1e-5 || u < 0 || v < 0 || u + v > 1) return false;
        output.t = t, output.u = u, output.v = v;
        return true;
    }

    glm::vec3 RayTrace(const RayIntersector & intersector, Ray ray, int maxDepth, bool enableShadow) {
        glm::vec3 color(0.0f);
        glm::vec3 weight(1.0f);

        for (int depth = 0; depth < maxDepth; depth++) {
            auto rayHit = intersector.IntersectRay(ray);
            if (! rayHit.IntersectState) return color;
            const glm::vec3 pos       = rayHit.IntersectPosition;//位置
            const glm::vec3 n         = rayHit.IntersectNormal;//法向
            const glm::vec3 kd        = rayHit.IntersectAlbedo;//反照率
            const glm::vec3 ks        = rayHit.IntersectMetaSpec;//吸收率
            const float     alpha     = rayHit.IntersectAlbedo.w;//透明度
            const float     shininess = rayHit.IntersectMetaSpec.w * 256;//高光衰减指数

            glm::vec3 result(0.0f);
            /******************* 2. Whitted-style ray tracing *****************/
            // your code here 计算环境光 
            glm::vec3 ambient = kd * intersector.InternalScene->AmbientIntensity;
            result += ambient;

            for (const Engine::Light & light : intersector.InternalScene->Lights) {
                glm::vec3 l;
                float     attenuation = 0.0f;
                /******************* 3. Shadow ray *****************/
                if (light.Type == Engine::LightType::Point) {
                    l           = light.Position - pos;
                    attenuation = 1.0f / glm::dot(l, l);
                    l = glm::normalize(l);
                    if (enableShadow) {
                        // your code here 启用阴影
                        Ray shadow_ray(pos, l);
                        while(true) {
                            auto hit = intersector.IntersectRay(shadow_ray);
                            if (!hit.IntersectState) break;//没被遮挡
                            if (glm::length(hit.IntersectPosition - pos) > glm::length(light.Position - pos)) break;
                            
                            float Alpha = hit.IntersectAlbedo.w;
                            if(Alpha >= 0.2f){
                                //被遮挡了
                                attenuation = 0.0f;
                                break;
                            }
                            else shadow_ray = Ray(hit.IntersectPosition, l);
                        }

                    }
                } else if (light.Type == Engine::LightType::Directional) {
                    l           = light.Direction;
                    attenuation = 1.0f;
                    l = glm::normalize(l);
                    if (enableShadow) {
                        // your code here
                        Ray shadow_ray(pos, l);
                        while(true) {
                            auto hit = intersector.IntersectRay(shadow_ray);
                            if (!hit.IntersectState) break;//没被遮挡
                            float Alpha = hit.IntersectAlbedo.w;
                            if(Alpha >= 0.2f){
                                //被遮挡了
                                attenuation = 0.0f;
                                break;
                            }
                            else {
                                shadow_ray = Ray(hit.IntersectPosition, l);
                            }
                        }
                    }
                }

                /******************* 2. Whitted-style ray tracing *****************/
                // your code here 计算高光和漫反射
                glm::vec3 diffuse = kd * light.Intensity * attenuation * glm::max(0.0f, glm::dot(n, l));
                result += diffuse;

                glm::vec3 v = glm::normalize(-ray.Direction);
                glm::vec3 h = glm::normalize(v + l);
                glm::vec3 specular = ks * light.Intensity * attenuation * glm::pow(glm::max(0.0f, glm::dot(n, h)), shininess);
                result += specular;
            }

            if (alpha < 0.9) {
                // refraction
                // accumulate color
                glm::vec3 R = alpha * glm::vec3(1.0f);
                color += weight * R * result;
                weight *= glm::vec3(1.0f) - R;

                // generate new ray
                ray = Ray(pos, ray.Direction);
            } else {
                // reflection
                // accumulate color
                glm::vec3 R = ks * glm::vec3(0.5f);
                color += weight * (glm::vec3(1.0f) - R) * result;
                weight *= R;

                // generate new ray
                glm::vec3 out_dir = ray.Direction - glm::vec3(2.0f) * n * glm::dot(n, ray.Direction);
                ray               = Ray(pos, out_dir);
            }
        }

        return color;
    }
} // namespace VCX::Labs::Rendering