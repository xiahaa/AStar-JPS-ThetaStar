#ifndef MAP_TYPES_H
#define MAP_TYPES_H

#include <vector>
#include <stdexcept> // Required for std::out_of_range

// Forward declaration for GridMap to be used in ISearch if necessary
// class GridMap; // Not strictly needed here yet, but good practice if ISearch will use it.

namespace pure_theta_star // Changed namespace
{

/**
 * @brief Represents a grid map for pathfinding algorithms.
 *
 * Stores the map as a 2D vector of integers, where 0 represents a traversable cell
 * and 100 represents an obstacle. Also stores dimensions and start/goal coordinates.
 */
class GridMap {
public:
    /**
     * @brief Constructs a GridMap.
     * @param map_data A 2D vector representing the grid. 0 for free, 100 for obstacle.
     * @param start_row The row index of the start point.
     * @param start_col The column index of the start point.
     * @param goal_row The row index of the goal point.
     * @param goal_col The column index of the goal point.
     */
    GridMap(const std::vector<std::vector<int>>& map_data,
            int start_row, int start_col,
            int goal_row, int goal_col)
        : data_(map_data),
          start_row_(start_row), start_col_(start_col),
          goal_row_(goal_row), goal_col_(goal_col) {
        if (map_data.empty()) {
            height_ = 0;
            width_ = 0;
        } else {
            height_ = map_data.size();
            width_ = map_data[0].size();
        }

        // Basic validation for start/goal points
        if (!CellOnGrid(start_row, start_col)) {
            throw std::out_of_range("Start point is off the grid.");
        }
        if (!CellOnGrid(goal_row, goal_col)) {
            throw std::out_of_range("Goal point is off the grid.");
        }
        if (CellIsObstacle(start_row, start_col)) {
             throw std::runtime_error("Start point is on an obstacle.");
        }
        if (CellIsObstacle(goal_row, goal_col)) {
            throw std::runtime_error("Goal point is on an obstacle.");
        }
    }

    /**
     * @brief Checks if a cell is within the grid boundaries.
     * @param r Row index.
     * @param c Column index.
     * @return True if the cell is on the grid, false otherwise.
     */
    bool CellOnGrid(int r, int c) const {
        return r >= 0 && r < height_ && c >= 0 && c < width_;
    }

    /**
     * @brief Checks if a cell is an obstacle.
     * Per user request: 0 is free, 100 is an obstacle.
     * @param r Row index.
     * @param c Column index.
     * @return True if the cell is an obstacle, false otherwise.
     * @throws std::out_of_range if the cell is not on the grid.
     */
    bool CellIsObstacle(int r, int c) const {
        if (!CellOnGrid(r, c)) {
            throw std::out_of_range("Cell coordinates are out of grid boundaries.");
        }
        return data_[r][c] == 100;
    }

    /**
     * @brief Gets the height of the map.
     * @return The height.
     */
    int getHeight() const { return height_; }

    /**
     * @brief Gets the width of the map.
     * @return The width.
     */
    int getWidth() const { return width_; }

    /**
     * @brief Gets the start row.
     * @return The start row index.
     */
    int getStartRow() const { return start_row_; }

    /**
     * @brief Gets the start column.
     * @return The start column index.
     */
    int getStartCol() const { return start_col_; }

    /**
     * @brief Gets the goal row.
     * @return The goal row index.
     */
    int getGoalRow() const { return goal_row_; }

    /**
     * @brief Gets the goal column.
     * @return The goal column index.
     */
    int getGoalCol() const { return goal_col_; }

private:
    std::vector<std::vector<int>> data_;
    int height_;
    int width_;
    int start_row_, start_col_;
    int goal_row_, goal_col_;
};

} // namespace pure_theta_star

#endif // MAP_TYPES_H
