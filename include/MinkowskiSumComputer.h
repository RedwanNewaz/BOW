//
// Created by airlab on 12/25/24.
//

#ifndef MBOW_MINKOWSKISUMCOMPUTER_H
#define MBOW_MINKOWSKISUMCOMPUTER_H
#include <vector>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/algorithms/convex_hull.hpp>
#include <boost/geometry/geometries/multi_point.hpp>
namespace bg = boost::geometry;
typedef bg::model::d2::point_xy<double> Point;
typedef bg::model::polygon<Point> Polygon;

class MinkowskiSumComputer {
private:
    Polygon circle_;
    Polygon rectangle_;
    Polygon minkowski_sum_;
    double grid_step_;

public:
    MinkowskiSumComputer(const Point& circle_center, double circle_radius,
                         const std::vector<Point>& rectangle_vertices,
                         double grid_step)
            : grid_step_(grid_step) {
        createCircle(circle_center, circle_radius);
        createRectangle(rectangle_vertices);
        computeMinkowskiSum();
    }

    void createCircle(const Point& center, double radius) {
        const int num_points = 36;  // Number of points to approximate the circle
        for (int i = 0; i < num_points; ++i) {
            double angle = 2.0 * M_PI * i / num_points;
            double x = center.x() + radius * std::cos(angle);
            double y = center.y() + radius * std::sin(angle);
            bg::append(circle_.outer(), Point(x, y));
        }
        bg::correct(circle_);
    }

    void createRectangle(const std::vector<Point>& vertices) {
        for (const auto& vertex : vertices) {
            bg::append(rectangle_.outer(), vertex);
        }
        bg::correct(rectangle_);
    }

    void computeMinkowskiSum() {
        std::vector<Point> minkowski_vertices;
        for (const auto& rect_point : rectangle_.outer()) {
            for (const auto& circle_point : circle_.outer()) {
                double x = rect_point.x() + circle_point.x();
                double y = rect_point.y() + circle_point.y();
                minkowski_vertices.emplace_back(x, y);
            }
        }
        bg::model::multi_point<Point> multi_point(minkowski_vertices.begin(), minkowski_vertices.end());
        bg::convex_hull(multi_point, minkowski_sum_);
    }

    std::vector<Point> computeCoordinatesInMinkowskiSum() {
        std::vector<Point> coordinates_inside;
        bg::model::box<Point> bbox;
        bg::envelope(minkowski_sum_, bbox);
        double minx = bg::get<bg::min_corner, 0>(bbox);
        double miny = bg::get<bg::min_corner, 1>(bbox);
        double maxx = bg::get<bg::max_corner, 0>(bbox);
        double maxy = bg::get<bg::max_corner, 1>(bbox);

        for (double x = minx; x <= maxx; x += grid_step_) {
            for (double y = miny; y <= maxy; y += grid_step_) {
                Point p(x, y);
                if (bg::within(p, minkowski_sum_)) {
                    coordinates_inside.push_back(p);
                }
            }
        }
        return coordinates_inside;
    }

    // Getter for the Minkowski sum polygon
    const Polygon& getMinkowskiSum() const {
        return minkowski_sum_;
    }
};
#endif //MBOW_MINKOWSKISUMCOMPUTER_H
