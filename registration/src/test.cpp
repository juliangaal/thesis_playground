#include "dsa.h"
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

TEST_CASE("Tests", "[tests]")
{
    {
        Eigen::Vector3f p1{0, 2, 2};
        REQUIRE(p1.norm() * p1.norm() == Catch::Approx(8.0f).epsilon(0.001));

        Eigen::Vector3f p2{1, 2, 2};
        auto dot = p1.dot(p2);
        REQUIRE(dot == Catch::Approx(8.f).epsilon(0.001));
    }
    {
        Eigen::Vector3f p1{1, 1, 1};
        Eigen::Vector3f p2{-1, -1, -1};
        REQUIRE(angle(p1, p2) == Catch::Approx(M_PI).epsilon(0.001));
    }
}
