// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>

static bool SSAA = true;
rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

//判断像素点是否在三角形内
//参数类型调整为浮点数
static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
   // 三角形内部的点满足：三角形的三个顶点到该点的向量的叉积方向相同
    Vector3f A = _v[0];
    Vector3f B = _v[1];
    Vector3f C = _v[2];
    //向量AB, BC, CA
    Vector3f AB = B - A;
    Vector3f BC = C - B;
    Vector3f CA = A - C;
   
    Vector3f P(x,y,1.0);

    Vector3f AP = P - A;
    Vector3f BP = P - B;
    Vector3f CP = P - C;

    Vector3f cp1 = AB.cross(AP);
    Vector3f cp2 = BC.cross(BP);
    Vector3f cp3 = CA.cross(CP);
   
    if (cp1.dot(cp2) > 0 && cp2.dot(cp3) > 0 && cp3.dot(cp1) > 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
    //SSAA
    if (SSAA)
    {
        for (int x = 0; x < width; x++)
        {
            for (int y = 0; y < height; y++)
            {
                //对采样值取平均值
                Eigen::Vector3f color(0.0, 0.0, 0.0);
                for (int index = 0; index < 4; index++)
                {
                    color += frame_buf_2SSAA[get_index(x, y)][index];
                }
                color /= 4.0;
                set_pixel(Eigen::Vector3f(x, y, 1.0), color);
            }
        }
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    //创建三角形的2维包围盒,即找到三角形x和y的最大值和最小值
    float min_x = std::min(v[0].x(), std::min(v[1].x(), v[2].x()));
    float max_x = std::max(v[0].x(), std::max(v[1].x(), v[2].x()));
    float min_y = std::min(v[0].y(), std::min(v[1].y(), v[2].y()));
    float max_y = std::max(v[0].y(), std::max(v[1].y(), v[2].y()));
    //整数化
    int min_x_int = std::floor(min_x);
    int max_x_int = std::ceil(max_x);
    int min_y_int = std::floor(min_y);
    int max_y_int = std::ceil(max_y);


    //遍历包围盒内的所有像素，然后使用像素中心的屏幕空间坐标来检查中心点是否在三角形内
    for (int x = min_x_int; x <= max_x_int; x++)
    {
        for (int y = min_y_int; y <= max_y_int; y++)
        {
            if (SSAA)
            {
                int index = 0;
                //设置像素内部采样点
                for (float sample_x = 0.25; sample_x < 1.0; sample_x += 0.5)
                {
                    for (float sample_y = 0.25; sample_y < 1.0; sample_y += 0.5)
                    {
                        //判断采样点是否被三角形覆盖
                        if (insideTriangle(x + sample_x, y + sample_y, t.v))
                        {
                            auto [alpha, beta, gamma] = computeBarycentric2D(x+sample_x, y+sample_y, t.v);
                            float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                            float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                            z_interpolated *= w_reciprocal;
                            if (z_interpolated < depth_buf_2SSAA[get_index(x, y)][index])
                            {
                                depth_buf_2SSAA[get_index(x, y)][index] = z_interpolated;
                                frame_buf_2SSAA[get_index(x, y)][index] = t.getColor();
                            } 
                        }
                        index++;
                    }
                }
            }
            else
            {
                //将像素中心点作为像素坐标
                if (insideTriangle((float)x + 0.5, (float)y + 0.5, t.v))
                {
                    //如果在内部，则将其位置处的插值深度值与深度缓冲区中的相应值进行比较
                    auto [alpha, beta, gamma] = computeBarycentric2D((float)x + 0.5, (float)y + 0.5, t.v);
                    float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;
                    
                    //深度值更小的像素覆盖深度值更大的像素
                    if (z_interpolated < depth_buf[get_index(x, y)])
                    {
                        Eigen::Vector3f p((float)x, (float)y, z_interpolated);
                        set_pixel(p, t.getColor());
                        //更新z值
                        depth_buf[get_index(x, y)] = z_interpolated;
                    }
                }

            }
           
        }
    }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});

        //SSAA
        for (int index = 0; index < frame_buf_2SSAA.size(); index++)
        {
            frame_buf_2SSAA[index].resize(4);
            std::fill(frame_buf_2SSAA[index].begin(), frame_buf_2SSAA[index].end(), Eigen::Vector3f{0,0,0});
        }
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());

        //SSAA
        for (int index = 0; index < depth_buf_2SSAA.size(); index++)
        {
            depth_buf_2SSAA[index].resize(4);
            std::fill(depth_buf_2SSAA[index].begin(), depth_buf_2SSAA[index].end(), std::numeric_limits<float>::infinity());
        }
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    //SSAA
    frame_buf_2SSAA.resize(w * h);
    depth_buf_2SSAA.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on