#ifndef CATMULL_ROM_PATH_PUBLISHER_POINT2D_HPP
#define CATMULL_ROM_PATH_PUBLISHER_POINT2D_HPP

namespace spline_creator {

struct Point2D {
    double x;  ///< The x-coordinate of the point.
    double y;  ///< The y-coordinate of the point.

    // Default constructor initializes the point to (0, 0)
    Point2D() : x(0.0), y(0.0) {}

    /**
     * @brief Constructs a point with given x and y values.
     * @param x_val The x-coordinate.
     * @param y_val The y-coordinate.
     */
    Point2D(double x_val, double y_val) : x(x_val), y(y_val) {}

    /**
     * @brief Overload of the addition operator.
     * @param other The other Point2D to add.
     * @return A new Point2D representing the sum.
     */
    Point2D operator+(const Point2D& other) const {
        return Point2D(x + other.x, y + other.y);
    }

    Point2D operator-(const Point2D& other) const {
        return Point2D(x - other.x, y - other.y);
    }

    /**
     * @brief Overload of the multiplication operator (scalar multiplication).
     * @param scalar The value to multiply with.
     * @return A new Point2D scaled by the given scalar.
     */
    Point2D operator*(double scalar) const {
        return Point2D(x * scalar, y * scalar);
    }

    /**
     * @brief Overload of the division operator (scalar division).
     * @param scalar The value to divide by.
     * @return A new Point2D scaled by the inverse of the given scalar.
     */
    Point2D operator/(double scalar) const {
        return Point2D(x / scalar, y / scalar);
    }

    /**
     * @brief Compound assignment operator for addition.
     * @param other The other Point2D to add.
     * @return A reference to this Point2D.
     */
    Point2D& operator+=(const Point2D& other) {
        x += other.x;
        y += other.y;
        return *this;
    }

    /**
     * @brief Compound assignment operator for subtraction.
     * @param other The other Point2D to subtract.
     * @return A reference to this Point2D.
     */
    Point2D& operator-=(const Point2D& other) {
        x -= other.x;
        y -= other.y;
        return *this;
    }

    /**
     * @brief Compound assignment operator for scalar multiplication.
     * @param scalar The value to multiply with.
     * @return A reference to this Point2D.
     */
    Point2D& operator*=(double scalar) {
        x *= scalar;
        y *= scalar;
        return *this;
    }

    /**
     * @brief Compound assignment operator for scalar division.
     * @param scalar The value to divide by.
     * @return A reference to this Point2D.
     */
    Point2D& operator/=(double scalar) {
        x /= scalar;
        y /= scalar;
        return *this;
    }
};

} // namespace spline_creator

#endif // CATMULL_ROM_PATH_PUBLISHER_POINT2D_HPP
