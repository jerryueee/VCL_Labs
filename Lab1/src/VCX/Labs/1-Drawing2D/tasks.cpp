#include <random>

#include <spdlog/spdlog.h>

#include "Labs/1-Drawing2D/tasks.h"

using VCX::Labs::Common::ImageRGB;

namespace VCX::Labs::Drawing2D {
    /******************* 1.Image Dithering *****************/
    void DitheringThreshold(
        ImageRGB &       output,
        ImageRGB const & input) {
        for (std::size_t x = 0; x < input.GetSizeX(); ++x)
            for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
                glm::vec3 color = input.At(x, y);
                output.At(x, y) = {
                    color.r > 0.5 ? 1 : 0,
                    color.g > 0.5 ? 1 : 0,
                    color.b > 0.5 ? 1 : 0,
                };
            }
    }

    void DitheringRandomUniform(
        ImageRGB &       output,
        ImageRGB const & input) {
        // your code here:
        std::default_random_engine e(static_cast<unsigned>(time(0)));
        std::uniform_real_distribution<float> rand(-0.5f, 0.5f);
        for (std::size_t x = 0; x < input.GetSizeX(); x++) {
            for (std::size_t y = 0; y < input.GetSizeY(); y++) {
                glm::vec3 color = input.At(x, y);
                float t = rand(e);//关键
                float r = std::clamp(color.r + t, 0.0f, 1.0f);
                float g = std::clamp(color.g + t, 0.0f, 1.0f);
                float b = std::clamp(color.b + t, 0.0f, 1.0f);
                output.At(x, y) = {
                    r > 0.5 ? 1.0f : 0.0f,
                    g > 0.5 ? 1.0f : 0.0f,
                    b > 0.5 ? 1.0f : 0.0f,
                };

            }
        }
        return;
    }

    void DitheringRandomBlueNoise(
        ImageRGB &       output,
        ImageRGB const & input,
        ImageRGB const & noise) {
        // your code here:
        for (std::size_t x = 0; x < input.GetSizeX(); x++) {
            for (std::size_t y = 0; y < input.GetSizeY(); y++) {
                glm::vec3 color = input.At(x, y);
                glm::vec3 t = noise.At(x, y);
                t -= 0.5;
                output.At(x, y) = {
                    color.r + t.r > 0.5 ? 1 : 0,
                    color.g + t.g > 0.5 ? 1 : 0,
                    color.b + t.b > 0.5 ? 1 : 0,
                };
            }
        }
        return;
    }

    void DitheringOrdered(
        ImageRGB &       output,
        ImageRGB const & input) {
        // your code here:
        int M[3][3] = {
            {6, 8, 4},
            {1, 0, 3},
            {5, 2, 7},
        };
        int width = input.GetSizeX(), height = input.GetSizeY();
        for (std::size_t x = 0; x < width; x++) {
            for (std::size_t y = 0; y < height; y++) {
                glm::vec3 color = input.At(x, y);
                float t = color.r + color.g + color.b;
                t /= (float) 3;
                for (std::size_t xx = 0; xx < 3; xx++) {
                    for (std::size_t yy = 0; yy < 3; yy++) {
                        std::size_t des_x = x * 3 + xx;
                        std::size_t des_y = y * 3 + yy;
                        float threshold = M[xx][yy] / (float)9;
                        if (t > threshold) output.At(des_x, des_y) = { 1, 1, 1, };
                        else output.At(des_x, des_y) = { 0, 0, 0, };
                    }
                }
            }
        }
        return;
    }

    void DitheringErrorDiffuse(
        ImageRGB &       output,
        ImageRGB const & input) {
        // your code here:
        std::size_t width = input.GetSizeX();
        std::size_t height = input.GetSizeY();
        ImageRGB img(input);
        for (std::size_t x = 0; x < width; x++) {
            for (std::size_t y = 0; y < height; y++) {
                glm:: vec3 color = img.At(x,y);
                output.At(x, y) = {
                    color.r > 0.5 ? 1.0f : 0.0f,
                    color.g > 0.5 ? 1.0f : 0.0f,
                    color.b > 0.5 ? 1.0f : 0.0f,
                };
                glm::vec3 after = output.At(x, y);
                glm::vec3 error = color - after;
                if (y < height - 1) {
                    glm::vec3 t = img.At(x, y + 1);
                    t += (7 / (float) 16) * error;
                    img.At(x, y + 1) = t;
                }
                if (x < width - 1) { 
                    glm::vec3 t = img.At(x + 1, y);
                    t += (5 / (float) 16) * error;
                    img.At(x + 1, y) = t;
                }
                if (y > 0 && x < width - 1) {
                    glm::vec3 t = img.At(x + 1, y - 1);
                    t += (3 / (float) 16) * error;
                    img.At(x + 1, y - 1) = t;
                }
                if (y < height - 1 && x < width - 1) {
                    glm::vec3 t = img.At(x+1,y+1);
                    t += (1 / (float)16) * error;
                    img.At(x + 1, y + 1) = t;
                }
            }
        }
        return;
    }

    /******************* 2.Image Filtering *****************/
    void Blur(
        ImageRGB &       output,
        ImageRGB const & input) {
        // your code here:
        float kernel[3][3] = {
            {1,1,1,},
            {1,1,1,},
            {1,1,1,},
        };
        int width = input.GetSizeX();
        int height = input.GetSizeY();
        for (int x = 0; x < width; x++) {
            for (int y = 0; y < height; y++) {
                glm::vec3 sum = { 0.0f, 0.0f, 0.0f };
                int xbegin = std::max(x - 1, 0), ybegin = std::max(y - 1, 0);
                int xend = std::min(x + 1, width - 1), yend = std::min(height - 1, y + 1);
                for (int i = xbegin; i <= xend; i++) {
                    for (int j = ybegin; j <= yend; j++) {
                        glm::vec3 tmp = input.At(i, j);
                        sum += tmp * kernel[1 + (i - x)][1 + (j - y)];
                    }
                }
                sum /= (float) 9;
                output.At(x, y) = sum;
            }
        }
        return;
    }

    void Edge(
        ImageRGB &       output,
        ImageRGB const & input) {
        // your code here:
        float Gx[3][3] = {
            {-1, 0, 1},
            {-2, 0, 2},
            {-1, 0, 1},
        };
        float Gy[3][3] = {
            { 1,  2,  1},
            { 0,  0,  0},
            {-1, -2, -1},
        };
        int width = input.GetSizeX();
        int height = input.GetSizeY();
        for (int x = 0; x < width; x++) {
            for (int y = 0; y < height; y++) {
                glm::vec3 sumx = { 0.0f, 0.0f, 0.0f };
                glm::vec3 sumy = { 0.0f, 0.0f, 0.0f };
                int xb = std::max(0, x - 1), xe = std::min(width - 1, x + 1);
                int yb = std::max(0, y - 1), ye = std::min(height - 1, y + 1);
                for (int i = xb; i <= xe; i++) {
                    for (int j = yb; j <= ye; j++) {
                        glm::vec3 tmp = input.At(i, j);
                        sumx += tmp * Gx[1 + (x - i)][1 + (y - j)]; 
                        sumy += tmp * Gy[1 + (x - i)][1 + (y - j)];
                    }
                }
                glm::vec3 color = glm::sqrt(sumx * sumx + sumy * sumy);
                output.At(x, y) = color;
            }
        }
        return;
    }

    /******************* 3. Image Inpainting *****************/
    void Inpainting(
        ImageRGB &         output,
        ImageRGB const &   inputBack,
        ImageRGB const &   inputFront,
        const glm::ivec2 & offset) {
        output             = inputBack;
        std::size_t width  = inputFront.GetSizeX();
        std::size_t height = inputFront.GetSizeY();
        glm::vec3 * g      = new glm::vec3[width * height];
        memset(g, 0, sizeof(glm::vec3) * width * height);
        // set boundary condition
        for (std::size_t y = 0; y < height; ++y) {
            // set boundary for (0, y), your code: g[y * width] = ?
            // set boundary for (width - 1, y), your code: g[y * width + width - 1] = ?
            g[y * width] = inputBack.At(offset.x, y + offset.y) - inputFront.At(0, y);
            g[y * width + width - 1] = inputBack.At(width - 1 + offset.x, y + offset.y) - inputFront.At(width - 1, y);
        }
        for (std::size_t x = 0; x < width; ++x) {
            // set boundary for (x, 0), your code: g[x] = ?
            // set boundary for (x, height - 1), your code: g[(height - 1) * width + x] = ?
            g[x] = inputBack.At(x + offset.x, offset.y) - inputFront.At(x, 0);
            g[(height - 1) * width + x] = inputBack.At(x + offset.x, height + offset.y - 1) - inputFront.At(x, height - 1);
        }

        // Jacobi iteration, solve Ag = b
        for (int iter = 0; iter < 8000; ++iter) {
            for (std::size_t y = 1; y < height - 1; ++y)
                for (std::size_t x = 1; x < width - 1; ++x) {
                    g[y * width + x] = (g[(y - 1) * width + x] + g[(y + 1) * width + x] + g[y * width + x - 1] + g[y * width + x + 1]);
                    g[y * width + x] = g[y * width + x] * glm::vec3(0.25);
                }
        }

        for (std::size_t y = 0; y < inputFront.GetSizeY(); ++y)
            for (std::size_t x = 0; x < inputFront.GetSizeX(); ++x) {
                glm::vec3 color = g[y * width + x] + inputFront.At(x, y);
                output.At(x + offset.x, y + offset.y) = color;
            }
        delete[] g;
        return;
    }

    /******************* 4. Line Drawing *****************/
    void DrawLine(
        ImageRGB &       canvas,
        glm::vec3 const  color,
        glm::ivec2 const p0,
        glm::ivec2 const p1) {
        // your code here:
        int x0 = p0.x, y0 = p0.y, x1 = p1.x, y1 = p1.y;
        if (x1 == x0) {
            int begin = y0 < y1 ? y0 : y1, end = y0 > y1 ? y0 : y1;
            for (begin; begin <= end; begin++) {
                canvas.At(x0, begin) = color;
            }
            return;
        }
        else if (y0 == y1) {
            int begin = x0 < x1 ? x0 : x1, end = x0 > x1 ? x0 : x1;
            for (begin; begin <= end; begin++) {
                canvas.At(begin, y0) = color;
            }
            return;
        }
        int dx = x1 - x0, dy = y1 - y0;
        int delta_x = dx > 0 ? 1 : -1, delta_y = dy > 0 ? 1 : -1;
        dx = abs(dx), dy = abs(dy);
        if (dx > dy) {
            //斜率绝对值小于1
            int F = 2 * dy - dx , y = y0, x = x0;//计算F时直接用绝对值即可，不需要考虑dx，dy的正负
            for (x ; x != x1 + delta_x; x += delta_x) {//x <= x1不能包含所有情况，导致错误
                canvas.At(x, y) = color;
                if (F < 0) F += 2 * dy;
                else {
                    y += delta_y;
                    F += 2 * (dy - dx);
                }
            }
        }
        else {
            int F = 2 * dx - dy, y = y0, x = x0;
            for (y; y != y1 + delta_y; y += delta_y) {
                canvas.At(x, y) = color;
                if (F < 0) F += 2 * dx;
                else {
                    x += delta_x;
                    F += 2 * (dx - dy);
                }
            }
        }
        return;
    }

    /******************* 5. Triangle Drawing *****************/
    //重载<，辅助Task 5核心函数中的排序
    bool operator<(glm::ivec2 const a, glm::ivec2 const b){
        if (a.y == b.y) return a.x < b.x;
        return a.y < b.y;
    }

    void DrawTriangleFilled(
        ImageRGB &       canvas,
        glm::vec3 const  color,
        glm::ivec2 const p0,
        glm::ivec2 const p1,
        glm::ivec2 const p2) {
        // your code here:
        //先将三个点按照y坐标从小到大排序
        glm::ivec2 point[3];
        point[0] = p0, point[1] = p1, point[2] = p2;
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 2 - i; j++) {
                if (point[j + 1] < point[j]) {
                    auto p = point[j];
                    point[j] = point[j + 1];
                    point[j + 1] = p;
                }
            }
        }
        int x0 = point[0].x, y0 = point[0].y, x1 = point[1].x, y1 = point[1].y, x2 = point[2].x, y2 = point[2].y;
        for (int y = y0; y <= y1; y++) {
            float m = 0,  n = 0; //关键：float，一开始写的int画出来是四边形，可能是线性插值时除法产生了误差，在扫描线循环时在取整误差会减小。
            if (y0 == y1) m = x0;
            else {
                m = ((x0 - x1) / (float) (y0 - y1)) * (y - y1) + x1; 
            }
            if (y0 == y2) n = x2;
            else {
                n = ((x0 - x2) / (float) (y0 - y2)) * (y - y2) + x2;
            }
            for (int i = std::min(m,n); i <= std::max(m,n); i++) canvas.At(i, y) = color;
        }
        for (int y = y1 + 1; y <= y2; y++) {
            float m = 0, n = 0;
            if (y1 == y2) m = x1;
            else {
                m = ((x1 - x2) / (float) (y1 - y2)) * (y - y1) + x1;
            }
            if (y0 == y2) n = x2;
            else {
                n = ((x0 - x2) / (float) (y0 - y2)) * (y - y2) + x2;
            }
            for (int i = std::min(m,n); i <= std::max(m,n); i++) canvas.At(i, y) = color;
        }
        return;
    }

    /******************* 6. Image Supersampling *****************/
    void Supersample(
        ImageRGB &       output,
        ImageRGB const & input,
        int              rate) {
        // your code here:
        int w = output.GetSizeX(), h = output.GetSizeY();
        int W = input.GetSizeX(), H = input.GetSizeY();
        for (int x = 0; x < w; x++) {
            for (int y = 0; y < h; y++) {
                //根据输入输出图像的比例output中的(x,y)对应到input相应的采样区域
                float X = x * (float) W / (float) w;
                float Y = y * (float) H / (float) h;
                glm::vec3 sum = { 0.0f, 0.0f, 0.0f };
                for (int xx = 0; xx < rate; xx++) {
                    for (int yy = 0; yy < rate; yy++) {
                        //+0.5f保证取得的值是图像中心值
                        float xi = X + (xx + 0.5f);
                        float yi = Y + (yy + 0.5f);
                        xi = glm::clamp(xi, 0.0f,(float) (W - 1));
                        yi = glm::clamp(yi, 0.0f,(float) (H - 1));
                        //线性插值
                        int x0 = (int) glm::floor(xi);
                        int x1 = glm::min(x0 + 1, W - 1);
                        int y0 = (int) glm::floor(yi);
                        int y1 = glm::min(y0 + 1, H - 1);
                        float t1 = xi - x0, t2 = yi - y0;
                        glm::vec3 color1 = input.At(x0, y0), color2 = input.At(x0, y1), color3 = input.At(x1, y1), color4 = input.At(x1, y0);
                        glm::vec3 tmp1 = (1 - t1) * color1 + t1 * color4;
                        glm::vec3 tmp2 = (1 - t1) * color2 + t1 * color3;
                        glm::vec3 color = (1 - t2) * tmp1 + t2 * tmp2;
                        sum += color;
                    }
                }
                output.At(x, y) = sum / (float) (rate * rate);
            }
        }
        return;
    }
    
    /******************* 7. Bezier Curve *****************/
    // Note: Please finish the function [DrawLine] before trying this part.
    glm::vec2 CalculateBezierPoint(
        std::span<glm::vec2> points,
        float const          t) {
        // your code here:
        if (points.size() == 1) return points.front();
        std::vector<glm::vec2> ret(points.size() - 1);
        for (int i = 0; i < points.size() - 1; i++) {
            glm::vec2 p, p1 = points[i], p2 = points[i + 1];
            p = (1 - t) * p1 + t * p2;
            ret[i] = p;
        }
        return CalculateBezierPoint(std::span<glm::vec2>(ret), t);
        //return glm::vec2 {0, 0};
    }
} // namespace VCX::Labs::Drawing2D