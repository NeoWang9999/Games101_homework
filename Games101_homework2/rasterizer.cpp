// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>
#include<numeric>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f>& positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return { id };
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i>& indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return { id };
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f>& cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return { id };
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(float x, float y, const Vector3f* _v)
{
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    //Vector3f P(x + 0.5f, y + 0.5f, 1.0f);
    Vector3f P(x, y, 1.0f);

    const Vector3f& A = _v[0];
    const Vector3f& B = _v[1];
    const Vector3f& C = _v[2];

    Vector3f AB = B - A;
    Vector3f BC = C - B;
    Vector3f CA = A - C;

    Vector3f AP = P - A;
    Vector3f BP = P - B;
    Vector3f CP = P - C;

    float z1 = AB.cross(AP).z();
    float z2 = BC.cross(BP).z();
    float z3 = CA.cross(CP).z();

    return (z1 > 0 && z2 > 0 && z3 > 0) || (z1 < 0 && z2 < 0 && z3 < 0);
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * y + v[1].x() * v[2].y() - v[2].x() * v[1].y()) / (v[0].x() * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * v[0].y() + v[1].x() * v[2].y() - v[2].x() * v[1].y());
    float c2 = (x * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * y + v[2].x() * v[0].y() - v[0].x() * v[2].y()) / (v[1].x() * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * v[1].y() + v[2].x() * v[0].y() - v[0].x() * v[2].y());
    float c3 = (x * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * y + v[0].x() * v[1].y() - v[1].x() * v[0].y()) / (v[2].x() * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * v[2].y() + v[0].x() * v[1].y() - v[1].x() * v[0].y());
    return { c1,c2,c3 };
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
        for (auto& vert : v)
        {
            vert.x() = 0.5 * width * (vert.x() + 1.0);
            vert.y() = 0.5 * height * (vert.y() + 1.0);
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
}

//Screen space rasterization
//void rst::rasterizer::rasterize_triangle(const Triangle& t) {
//    auto v = t.toVector4();
//
//    // TODO : Find out the bounding box of current triangle.
//    float minx = std::min({ v[0].x(), v[1].x(), v[2].x() });
//    float miny = std::min({ v[0].y(), v[1].y(), v[2].y() });
//    float maxx = std::max({ v[0].x(), v[1].x(), v[2].x() });
//    float maxy = std::max({ v[0].y(), v[1].y(), v[2].y() });
//    // iterate through the pixel and find if the current pixel is inside the triangle
//
//    for (int x = (int)minx; x < maxx; x++)
//    {
//        for (int y = (int)miny; y < maxy; y++)
//        {
//
//            if (!insideTriangle(x, y, t.v)) continue;
//            // If so, use the following code to get the interpolated z value.
//            auto [alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
//            float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
//            float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
//            z_interpolated *= w_reciprocal;
//
//            int buf_index = get_index(x, y);
//
//            if (z_interpolated >= depth_buf[buf_index]) continue;
//
//            depth_buf[buf_index] = z_interpolated;
//            
//            // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
//            set_pixel(Vector3f(x, y, 1), t.getColor());
//        }
//    }
//}



//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    // with ssaa

    auto v = t.toVector4();

    // TODO : Find out the bounding box of current triangle.
    float minx = std::min({ v[0].x(), v[1].x(), v[2].x() });
    float miny = std::min({ v[0].y(), v[1].y(), v[2].y() });
    float maxx = std::max({ v[0].x(), v[1].x(), v[2].x() });
    float maxy = std::max({ v[0].y(), v[1].y(), v[2].y() });
    // iterate through the pixel and find if the current pixel is inside the triangle
    for (int x = (int)minx; x <= maxx; x++)
    {
        for (int y = (int)miny; y <= maxy; y++)
        {
            bool isSetPixel = false;
            for (float i = 0.; i < 1.; i += 1. / ssaa_w) {
                for (float j = 0.; j < 1; j += 1. / ssaa_h) {
                    Vector3f subP(x + i, y + j, 0);
                    if (!insideTriangle(subP.x(), subP.y(), t.v)) {
                        continue;
                    }
                    // If so, use the following code to get the interpolated z value.
                    auto [alpha, beta, gamma] = computeBarycentric2D(subP.x(), subP.y(), t.v);
                    float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;

                    int ssaa_index = get_ssaa_index(x + i, y + j);

                    if (z_interpolated >= depth_buf[ssaa_index]) {
                        continue;
                    }
                    depth_buf[ssaa_index] = z_interpolated;
                    color_buf[ssaa_index] = t.getColor();

                    isSetPixel = true;
                }
            }
            if (!isSetPixel){
                continue;
            }
            int buf_index = get_index(x, y);
            Vector3f comb_color = Vector3f::Zero();

            for (float i = 0.; i < 1.; i += 1. / ssaa_w) {
                for (float j = 0.; j < 1; j += 1. / ssaa_h) {
                    int ssaa_index = get_ssaa_index(x + i, y + j);
                    comb_color += (1. / (ssaa_w * ssaa_h)) * color_buf[ssaa_index];
                }
            }
            Vector3f cur_point = Vector3f(x, y, 0);
            set_pixel(cur_point, comb_color);
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
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{ 0, 0, 0 });
        std::fill(color_buf.begin(), color_buf.end(), Eigen::Vector3f{ 0, 0, 0 });
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    //depth_buf.resize(w * h);
    depth_buf.resize(w * h * ssaa_w * ssaa_h);
    color_buf.resize(w * h * ssaa_w * ssaa_h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height - 1 - y) * width + x;
}

int rst::rasterizer::get_ssaa_index(float x, float y)
{
    return (height * ssaa_h - 1 - y * ssaa_h) * width * ssaa_w + x * ssaa_w;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    //auto ind = (height - 1 - point.y()) * width + point.x();
    auto ind = get_index(point.x(), point.y());
    frame_buf[ind] = color;

}

// clang-format on