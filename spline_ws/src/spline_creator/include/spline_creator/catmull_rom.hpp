#ifndef CATMULL_ROM_PATH_PUBLISHER_CATMULL_ROM_HPP
#define CATMULL_ROM_PATH_PUBLISHER_CATMULL_ROM_HPP

#include <vector>
#include "spline_creator/point2d.hpp"

namespace spline_creator {

/**
 * @brief Class for computing a centripetal Catmull–Rom spline.
 *
 * This class provides functions to compute a spline segment that interpolates
 * between four control points using the centripetal parameterization.
 */
class CatmullRomSpline {
public:
    /**
     * @brief Interpolates a spline segment between P1 and P2.
     *
     * Given four control points P0, P1, P2, and P3, computes the spline curve defined
     * for t between t1 and t2 (i.e., curve segment between P1 and P2). The resulting curve
     * is re-sampled to have points approximately "resolution" meters apart.
     *
     * @param P0 The first control point.
     * @param P1 The second control point (start of the curve segment).
     * @param P2 The third control point (end of the curve segment).
     * @param P3 The fourth control point.
     * @param resolution Desired approximate distance between points along the spline.
     * @return A vector of Point2D representing the discretized spline segment.
     */
    static std::vector<Point2D> interpolateSegment(const Point2D& P0,
                                                   const Point2D& P1,
                                                   const Point2D& P2,
                                                   const Point2D& P3,
                                                   double resolution);

private:
    /**
     * @brief Computes an interpolated point on the Catmull-Rom spline.
     *
     * Uses the centripetal Catmull-Rom formulation.
     *
     * @param P0 First control point.
     * @param P1 Second control point.
     * @param P2 Third control point.
     * @param P3 Fourth control point.
     * @param t  The parameter value at which the point is evaluated.
     * @param t0 Parameter corresponding to P0.
     * @param t1 Parameter corresponding to P1.
     * @param t2 Parameter corresponding to P2.
     * @param t3 Parameter corresponding to P3.
     * @return The interpolated point.
     */
    static Point2D interpolate(const Point2D& P0, const Point2D& P1,
                               const Point2D& P2, const Point2D& P3,
                               double t, double t0, double t1, double t2, double t3);

    /**
     * @brief Computes the next parameter value for the centripetal formulation.
     *
     * Given a previous parameter value t_i and control points P_i and P_j, computes
     * t_j using the centripetal parameterization (alpha = 0.5).
     *
     * @param t_i Previous parameter value.
     * @param P_i Previous control point.
     * @param P_j Current control point.
     * @return The computed parameter t_j.
     */
    static double tj(double t_i, const Point2D& P_i, const Point2D& P_j);
};

} // namespace spline_creator

#endif // CATMULL_ROM_PATH_PUBLISHER_CATMULL_ROM_HPP
