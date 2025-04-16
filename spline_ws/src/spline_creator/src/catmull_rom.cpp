#include "spline_creator/catmull_rom.hpp"
#include <cmath>
#include <iostream>

namespace spline_creator {

// Computes the next parameter value t_j using the centripetal method (alpha = 0.5)
double CatmullRomSpline::tj(double t_i, const Point2D& P_i, const Point2D& P_j) {
    // Compute Euclidean distance between points.
    double dx = P_j.x - P_i.x;
    double dy = P_j.y - P_i.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    // alpha = 0.5 for centripetal parameterization.
    return t_i + std::pow(distance, 0.5);
}

// Computes a point on the spline at parameter value t
Point2D CatmullRomSpline::interpolate(const Point2D& P0, const Point2D& P1,
                                        const Point2D& P2, const Point2D& P3,
                                        double t, double t0, double t1, double t2, double t3) {
    // First-level interpolation
    Point2D A1 = P0 * ((t1 - t) / (t1 - t0)) + P1 * ((t - t0) / (t1 - t0));
    Point2D A2 = P1 * ((t2 - t) / (t2 - t1)) + P2 * ((t - t1) / (t2 - t1));
    Point2D A3 = P2 * ((t3 - t) / (t3 - t2)) + P3 * ((t - t2) / (t3 - t2));

    // Second-level interpolation
    Point2D B1 = A1 * ((t2 - t) / (t2 - t0)) + A2 * ((t - t0) / (t2 - t0));
    Point2D B2 = A2 * ((t3 - t) / (t3 - t1)) + A3 * ((t - t1) / (t3 - t1));

    // Final interpolation to get the point on the curve
    Point2D C = B1 * ((t2 - t) / (t2 - t1)) + B2 * ((t - t1) / (t2 - t1));
    return C;
}

// Interpolates the spline segment between P1 and P2 and re-samples it at approximately 'resolution'
std::vector<Point2D> CatmullRomSpline::interpolateSegment(const Point2D& P0,
                                                            const Point2D& P1,
                                                            const Point2D& P2,
                                                            const Point2D& P3,
                                                            double resolution) {
    // Parameterize the control points
    double t0 = 0.0;
    double t1 = tj(t0, P0, P1);
    double t2 = tj(t1, P1, P2);
    double t3 = tj(t2, P2, P3);

    // Sample the spline using a fixed number of steps (adjustable)
    int num_samples = 100;  // Use a high number to approximate the arc length well.
    std::vector<Point2D> samples;
    for (int i = 0; i <= num_samples; i++) {
        // Interpolate between t1 and t2
        double t = t1 + (t2 - t1) * (static_cast<double>(i) / num_samples);
        samples.push_back(interpolate(P0, P1, P2, P3, t, t0, t1, t2, t3));
    }

    // Compute cumulative arc lengths along the sampled points.
    std::vector<double> cumlength;
    cumlength.push_back(0.0);
    for (size_t i = 1; i < samples.size(); i++) {
        double dx = samples[i].x - samples[i - 1].x;
        double dy = samples[i].y - samples[i - 1].y;
        double d = std::sqrt(dx * dx + dy * dy);
        cumlength.push_back(cumlength.back() + d);
    }

    // Resample the curve to achieve approximately "resolution" meter spacing.
    std::vector<Point2D> resampled;
    resampled.push_back(samples.front());
    double current_length = resolution;
    for (size_t i = 1; i < samples.size(); i++) {
        // Interpolate along the sample segment if the cumulative length passes the desired spacing.
        while (current_length < cumlength[i] && (cumlength[i] - cumlength[i - 1]) > 0.0) {
            double segment_length = cumlength[i] - cumlength[i - 1];
            double ratio = (current_length - cumlength[i - 1]) / segment_length;
            // Linear interpolation between samples[i-1] and samples[i]
            Point2D interp_point = samples[i - 1] * (1.0 - ratio) + samples[i] * ratio;
            resampled.push_back(interp_point);
            current_length += resolution;
        }
    }
    // Ensure the final point is included.
    if (!(resampled.back().x == samples.back().x && resampled.back().y == samples.back().y)) {
        resampled.push_back(samples.back());
    }
    return resampled;
}

} // namespace spline_creator
