#pragma once

#include <vector>
#include <fstream>
#include <boost/filesystem.hpp>

// FLANN deps
#include <flann/flann.hpp>

#include "util.h"

namespace ply
{

    class PLYWriter
    {
    public:
        PLYWriter(boost::filesystem::path& filepath);
        ~PLYWriter();
        PLYWriter& operator=(const PLYWriter& other) = delete;
        PLYWriter& operator=(PLYWriter&&) = delete;
        PLYWriter(const PLYWriter&) = delete;
        PLYWriter(PLYWriter&&) = delete;

        void write(const std::vector<util::Point>& point, const std::vector<util::Point>& normals);
        void write(const flann::Matrix<double>& points, size_t number_of_points, const std::vector<util::Point>& normals);

        void writeMesh(const std::vector<Eigen::Vector3d>& vertices, const std::vector<Eigen::Vector3i>& indices);

    private:
        void write(const util::Point& point, const util::Point& normal);
        void dot_to_comma(std::string& s);
        void write_properties(const std::vector<util::Point>& point);
        void write_properties(size_t number_of_points);

        void write_line(const std::string& line);
        std::ofstream filestream_;
        bool header_complete_;
        const std::string locale_;
    };

}

