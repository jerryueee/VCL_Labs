#include "Labs/5-Visualization/tasks.h"

#include <numbers>

using VCX::Labs::Common::ImageRGB;
namespace VCX::Labs::Visualization {

    struct CoordinateStates {
        // your code here
        int total = 0;
        std::vector<float> mileage;
        std::vector<int> cylinders;
        std::vector<float> displacement;
        std::vector<float> horsepower;
        std::vector<float> weight;
        std::vector<float> acceleration;
        std::vector<int> year;
        CoordinateStates(std::vector<Car> const& data){
            total = data.size();
            for(int i = 0; i < total; i++){
                mileage.push_back(data[i].mileage);
                cylinders.push_back(data[i].cylinders);
                displacement.push_back(data[i].displacement);
                horsepower.push_back(data[i].horsepower);
                weight.push_back(data[i].weight);
                acceleration.push_back(data[i].acceleration);
                year.push_back(data[i].year);
            }
        }
        void Paint(Common::ImageRGB & input){
            int width = input.GetSizeX(), height = input.GetSizeY();
            //printf("%d,%d", width, height);
            float pos[7] = {0.05, 0.20, 0.35, 0.50, 0.65, 0.80, 0.95};
            std::string text1[7] = {"cylinders", "displacement", "weight", "horsepower", "acceleration(0-60) mph", "mileage", "year"};
            std::string text2[7] = {"9", "494", "5493", "249", "27", "51", "84"};
            std::string text3[7] = {"2", "29", "1260", "27", "6", "5", "68"};
            for(int i = 0; i < 7; i++){
                PrintText(input, glm::vec4(0, 0, 0, 1), glm::vec2(pos[i], 0.01), 0.01f, text1[i]);
            }
            for(int i = 0; i < 7; i++){
                PrintText(input, glm::vec4(0, 0, 0, 1), glm::vec2(pos[i], 0.04), 0.01f, text2[i]);
            }
            for(int i = 0; i < 7; i++){
                PrintText(input, glm::vec4(0, 0, 0, 1), glm::vec2(pos[i], 0.98), 0.01f, text3[i]);
            }
            for(int i = 0; i < 7; i++){
                DrawRect(input, glm::vec4(0.957, 0.957, 0.957, 0.957), glm::vec2(pos[i] - 0.01, 0.07), glm::vec2(0.02, 0.89), 0.01f);
                for(int x = (pos[i] - 0.01) * width; x <= (pos[i] + 0.01) * width; x++){
                    for(int y = 0.07 * height; y <= 0.96 * height; y++){
                        input.At(x, y) = glm::vec3(0.957, 0.957, 0.957);
                    }
                }
            }
            for(int i = 0; i < 7; i++){
                DrawLine(input, glm::vec4(0.5, 0.5, 0.5, 1), glm::vec2(pos[i], 0.07), glm::vec2(pos[i], 0.96), 0.01f);
            }
            //glm::vec3 color = glm::vec3(0.5, 0.539, 0.527);
            //glm::vec3 color = glm::vec3(0.367, 0.148, 0.070);
            //glm::vec3 color = glm::vec3(0.527, 0.805, 0.918);
            //glm::vec3 warm = glm::vec3(0.527, 0.199, 0.141);
            //glm::vec3 warm = glm::vec3(0.527, 0.148, 0.340);//草莓色    
            //glm::vec3 warm = glm::vec3(176, 48, 96) / (float)256;//栗色      
            glm::vec3 warm = glm::vec3(148, 42, 42) / (float)256;//棕色
            //glm::vec3 cold = glm::vec3(0.527, 0.895, 0.918);天蓝
            //glm::vec3 cold = glm::vec3(0.238, 0.348, 0.668);//钴色
            //glm::vec3 cold = glm::vec3(0.199, 0.629, 0.785);//孔雀蓝
            //glm::vec3 cold = glm::vec3(0.6875, 0.875, 0.898);//浅灰兰色
            //glm::vec3 cold = glm::vec3(65, 105, 225) / (float)256;//品蓝
            glm::vec3 cold = glm::vec3(176, 196, 222) / (float)256;//亮钢蓝

            for(int i = 0; i  < total; i++){
                //cylinders
                glm::vec2 p1 = glm::vec2(0.05, 0.07 + (float)(9 - cylinders[i]) / (9 - 2) * 0.89);
                //displacement
                glm::vec2 p2 = glm::vec2(0.20, 0.07 + (float)(494 - displacement[i]) / (494 - 29) * 0.89);
                //weight
                glm::vec2 p3 = glm::vec2(0.35, 0.07 + (float)(5493 - weight[i]) / (5493 - 1260) * 0.89);
                //horsepower
                glm::vec2 p4 = glm::vec2(0.50, 0.07 + (float)(249 - horsepower[i]) / (249 - 27) * 0.89);
                //acceleration
                glm::vec2 p5 = glm::vec2(0.65, 0.07 + (float)(27 - acceleration[i]) / (27 - 6) * 0.89);
                //mileage
                glm::vec2 p6 = glm::vec2(0.80, 0.07 + (float)(51 - mileage[i]) / (51 - 5) * 0.89);
                //year
                glm::vec2 p7 = glm::vec2(0.95, 0.07 + (float)(84 - year[i]) / (84 - 68) * 0.89);
                
                float t = (float) i / total;
                glm::vec3 color = warm * t + cold * (1 - t);

                DrawLine(input, glm::vec4(color, 1), p1, p2, 0.005);
                DrawLine(input, glm::vec4(color, 1), p2, p3, 0.005);
                DrawLine(input, glm::vec4(color, 1), p3, p4, 0.005);
                DrawLine(input, glm::vec4(color, 1), p4, p5, 0.005);
                DrawLine(input, glm::vec4(color, 1), p5, p6, 0.005);
                DrawLine(input, glm::vec4(color, 1), p6, p7, 0.005);
            }
        }
    };

    bool PaintParallelCoordinates(Common::ImageRGB & input, InteractProxy const & proxy, std::vector<Car> const & data, bool force) {
        // your code here
        // for example: 
        //   static CoordinateStates states(data);
        //   SetBackGround(input, glm::vec4(1));
        //   ...
        static CoordinateStates states(data);
        SetBackGround(input, glm::vec4(1));
        states.Paint(input);
        return true;
    }

    void LIC(ImageRGB & output, Common::ImageRGB const & noise, VectorField2D const & field, int const & step) {
        // your code here
        int width = output.GetSizeX(), height = output.GetSizeY();
        for(int i = 0; i < width; i++){
            for(int j = 0; j < height; j++){
                //正方向卷积
                float x = i, y = j;
                glm::vec3 forward_sum = glm::vec3(0);
                float weight_forward = 0;
                for(int k = 0; k < step; k++){
                    glm::vec2 v = field.At(std::round(x), std::round(y));
                    float dx = v.x, dy = v.y;
                    float dt_x = 0.0, dt_y = 0.0;
                    float dt;
                    if (dx > 0) dt_x = ((std::floor(x) + 1) - x) / dx;
                    else if (dx < 0) dt_x = (x - (std::ceil(x) - 1)) / -dx;
                    if (dy > 0) dt_y = ((std::floor(y) + 1) - y) / dy;
                    else if (dy < 0) dt_y = (y - (std::ceil(y) - 1)) / -dy;
                    if(dx == 0 && dy == 0) dt = 0.0f;
                    else dt = std::min(dt_x, dt_y);
                    x = std::min(std::max(x + dt * dx, 0.0f), (float)width - 1);
                    y = std::min(std::max(y + dt * dy, 0.0f), (float)height - 1);
                    float weight = glm::exp(- std::pow(k,2) / (2.0f * step * step));//高斯核
                    //float weight = glm::pow(glm::cos(1 + k * 0.46), 2);//cos
                    weight_forward += weight;
                    forward_sum += noise.At(std::round(x), std::round(y)) * weight;
                }
                //逆方向卷积
                x = i, y = j;
                glm::vec3 backward_sum = glm::vec3(0);
                float weight_backward = 0;
                for(int k = 0; k < step; k++){
                    glm::vec2 v = field.At(std::round(x), std::round(y));
                    float dx = -v.x, dy = -v.y;
                    float dt_x = 0.0, dt_y = 0.0;
                    float dt;
                    if (dx > 0) dt_x = ((std::floor(x) + 1) - x) / dx;
                    else if (dx < 0) dt_x = (x - (std::ceil(x) - 1))/ -dx;
                    if (dy > 0) dt_y = ((std::floor(y) + 1) - y) / dy;
                    else if (dy < 0) dt_y = (y - (std::ceil(y) - 1)) / -dy;
                    if(dx == 0 && dy == 0) dt = 0.0f;
                    else dt = std::min(dt_x, dt_y);
                    x = std::min(std::max(x + dt * dx, 0.0f), (float)width - 1);
                    y = std::min(std::max(y + dt * dy, 0.0f), (float)height - 1);
                    //float weight = glm::pow(glm::cos(1 +  k * 0.46), 2);//cos
                    float weight = glm::exp(- std::pow(k,2) / (2.0f * step * step));//高斯核
                    weight_backward += weight;
                    backward_sum += noise.At(std::round(x), std::round(y)) * weight;
                }
                output.At(i, j) = (forward_sum + backward_sum) / (weight_forward + weight_backward);
            }
        }
        return ;
    }
}; // namespace VCX::Labs::Visualization