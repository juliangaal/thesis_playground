#include "ply.h"
#include <iostream>

using namespace ply;

PLYWriter::PLYWriter(boost::filesystem::path& filepath)
        : filestream_()
        , header_complete_(false)
        , locale_(std::locale("").name())
{
    if (filepath.empty())
    {
        throw std::runtime_error("Can't open ply filestream: empty path\n");
    }

    if (!filepath.has_extension())
    {
        throw std::runtime_error("Can't open ply filestream. Make sure to use '.ply' extension");
    }

    if (filepath.extension().string() != ".ply")
    {
        std::cout << "WARNING: replacing ply file extension from " << filepath.extension() << " to '.ply'\n";
        filepath.replace_extension(".ply");
    }

    filestream_.open(filepath.string());
    if (!filestream_.is_open())
    {
        throw std::runtime_error("Can't open ply filestream for .ply file");
    }

    if (locale_.empty())
    {
        throw std::runtime_error("Can't find user locale\n");
    }

    write_line("ply");
    write_line("format ascii 1.0");
    write_line("comment 3ds SS2021 gruppe 2");
}

PLYWriter::~PLYWriter()
{
    filestream_.close();
}

void PLYWriter::write_line(const std::string& line)
{
    filestream_ << line << "\n";
}

void PLYWriter::write_properties(const std::vector<util::Point>& points)
{
    if (!header_complete_)
    {
        write_line(std::string("element vertex ") + std::to_string(points.size()));
        write_line("property double x");
        write_line("property double y");
        write_line("property double z");

        write_line("property double nx");
        write_line("property double ny");
        write_line("property double nz");

        write_line("end_header");
        header_complete_ = true;
    }
}

void PLYWriter::write_properties(size_t number_of_points)
{
    if (!header_complete_)
    {
        write_line(std::string("element vertex ") + std::to_string(number_of_points));
        write_line("property double x");
        write_line("property double y");
        write_line("property double z");

        write_line("property double nx");
        write_line("property double ny");
        write_line("property double nz");

        write_line("end_header");
        header_complete_ = true;
    }
}

void PLYWriter::dot_to_comma(std::string& s)
{
    // With german locale, don't replace . with ,. Y is ply like this
    if (locale_.find("de_DE.UTF-8") != std::string::npos)
    {
        return;
    }

    std::replace(s.begin(), s.end(), '.', ',');
}

void PLYWriter::write(const util::Point& point, const util::Point& normal)
{
    std::string point_s = std::to_string(point[0]) + " " + std::to_string(point[1]) + " " + std::to_string(point[2]) + " " + std::to_string(normal[0]) + " " + std::to_string(normal[1]) + " " + std::to_string(normal[2]);
    dot_to_comma(point_s);
    write_line(point_s);
}

void PLYWriter::write(const std::vector<util::Point>& points, const std::vector<util::Point>& normals)
{
    if (points.size() != normals.size())
    {
        std::cerr << "Points and colors must have the same size!" << std::endl;
        return;
    }

    write_properties(points);

    //for (const auto& point: points)
    for (auto index = 0u; index < points.size(); ++index)
    {
        write(points[index], normals[index]);
    }
}

void PLYWriter::write(const flann::Matrix<double>& points, size_t number_of_points, const std::vector<util::Point>& normals)
{
    if (number_of_points != normals.size())
    {
        std::cerr << "Points and colors must have the same size!" << std::endl;
        return;
    }

    write_properties(number_of_points);

    util::Point point;

    //for (const auto& point: points)
    for (auto index = 0u; index < number_of_points; ++index)
    {
        point[0] = points[index][0];
        point[1] = points[index][1];
        point[2] = points[index][2];

        write(point, normals[index]);
    }
}

void PLYWriter::writeMesh(const std::vector<Eigen::Vector3d>& vertices, const std::vector<Eigen::Vector3i>& indices)
{
    write_line(std::string("element vertex ") + std::to_string(vertices.size()));
    write_line("property double x");
    write_line("property double y");
    write_line("property double z");

    write_line(std::string("element face ") + std::to_string(indices.size()));

    write_line("property list uchar int vertex_indices");

    write_line("end_header");

    for (const auto& vertex : vertices)
    {
        std::string point_s = std::to_string(vertex[0]) + " " + std::to_string(vertex[1]) + " " + std::to_string(vertex[2]);
        dot_to_comma(point_s);
        write_line(point_s);
    }

    for (const auto& index : indices)
    {
        std::string point_s = "3 " + std::to_string(index[0]) + " " + std::to_string(index[1]) + " " + std::to_string(index[2]);
        dot_to_comma(point_s);
        write_line(point_s);
    }
}
