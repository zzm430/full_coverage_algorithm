//
// Copyright [2020] Nobleo Technology"  [legal/copyright]
//
#include <algorithm>
#include <full_coverage_path_planner/common.h>
#include <iostream>
#include <limits>
#include <list>
#include <vector>

int distanceToClosestPoint(Point_t poi, std::list<Point_t> const& goals)
{
    // Return minimum distance from goals-list
    int min_dist = INT_MAX;
    // POI starting point
    //??????????????????????? is the list of goals, one block at a time? (distance is reading all the cells)
    //??????????? where does the Goal list come from and it's entirely not sorted? Also, why distance squared not
    // regular distance?
    std::list<Point_t>::const_iterator it;
    for (it = goals.begin(); it != goals.end(); ++it)
    {
        int cur_dist = distanceSquared((*it), poi);
        if (cur_dist < min_dist)
        {
            min_dist = cur_dist;
        }
    }
    return min_dist;
}

int distanceSquared(const Point_t& p1, const Point_t& p2)
{
    int dx = p2.x - p1.x;
    int dy = p2.y - p1.y;

    int dx2 = dx * dx;
    if (dx2 != 0 && dx2 / dx != dx)
    {
        throw std::range_error("Integer overflow error for the given points");
    }

    int dy2 = dy * dy;
    if (dy2 != 0 && dy2 / dy != dy)
    {
        throw std::range_error("Integer overflow error for the given points");
    }

    if (dx2 > std::numeric_limits<int>::max() - dy2)
        throw std::range_error("Integer overflow error for the given points");
    int d2 = dx2 + dy2;

    return d2;
}

/**
 * Sort vector<gridNode> by the heuristic value of the last element
 * @return whether last elem. of first has a larger heuristic value than last elem of second
 */
// ????????????????????? why do we need to compare last element of first vs second? also, why not in .h file?
bool sort_gridNodePath_heuristic_desc(const std::vector<gridNode_t>& first, const std::vector<gridNode_t>& second)
{
    return (first.back().he > second.back().he);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//?????????????????????? don't know how this is done...
bool a_star_to_open_space(std::vector<std::vector<bool>> const& grid, gridNode_t init, int cost,
                          std::vector<std::vector<bool>>& visited, std::list<Point_t> const& open_space,
                          std::list<gridNode_t>& pathNodes)
{
    uint dx, dy, dx_prev, nRows = grid.size(), nCols = grid[0].size();

    std::vector<std::vector<bool>> closed(nRows, std::vector<bool>(nCols, eNodeOpen));
    //?????????? can you help me understand this initialization... with () in middle??  matrix of values (why
    // nrows/std::vector<bool>

    // All nodes in the closest list are currently still open

    closed[init.pos.y][init.pos.x] = eNodeVisited;  // Of course we have visited the current/initial location
#ifdef DEBUG_PLOT
    std::cout << "A*: Marked init " << init << " as eNodeVisited (true)" << std::endl;
    printGrid(closed);
#endif

    std::vector<std::vector<gridNode_t>> open1(1, std::vector<gridNode_t>(1, init));  // open1 is a *vector* of paths
    //????? how does this work???? where does this vector of path gets generated??

    while (true)
    {
#ifdef DEBUG_PLOT
        std::cout << "A*: open1.size() = " << open1.size() << std::endl;
#endif
        if (open1.size() == 0)  // If there are no open paths, there's no place to go and we must resign
        {
            // Empty end_node list and add init as only element
            pathNodes.erase(pathNodes.begin(), --(pathNodes.end()));
            pathNodes.push_back(init);
            // What does this do push init back???????????????????? do we just not go anywhere? (what does resign mean?)
            return true;  // We resign, cannot find a path
        }
        else
        {
            // ????????????Sort elements from high to low (because sort_gridNodePath_heuristic_desc uses a > b)
            std::sort(open1.begin(), open1.end(), sort_gridNodePath_heuristic_desc);
            //???????? what's the difference between Open1.end vs open1.back?
            std::vector<gridNode_t> nn = open1.back();  // Get the *path* with the lowest heuristic cost
            open1.pop_back();  // The last element is no longer open because we use it here, so remove from open list
#ifdef DEBUG_PLOT
            std::cout << "A*: Check out path from" << nn.front().pos << " to " << nn.back().pos << " of length "
                      << nn.size() << std::endl;
#endif

            // Does the path nn end in open space?
            if (visited[nn.back().pos.y][nn.back().pos.x] == eNodeOpen)
            {
                // If so, we found a path to open space
                // Copy the path nn to pathNodes so we can report that path (to get to open space)
                std::vector<gridNode_t>::iterator iter;
                for (iter = nn.begin(); iter != nn.end(); ++iter)
                {
                    pathNodes.push_back((*iter));
                }

                return false;  // We do not resign, we found a path
            }
            else  // Path nn does not lead to open space
            {
                if (nn.size() > 1)
                {
                    // Create iterator for gridNode_t list and let it point to the last element of nn
                    std::vector<gridNode_t>::iterator it = --(nn.end());
                    dx = it->pos.x - (it - 1)->pos.x;
                    dy = it->pos.y - (it - 1)->pos.y;
                    // TODO(CesarLopez) docs: this seems to cycle through directions
                    // (notice the shift-by-one between both sides of the =)
                    dx_prev = dx;
                    dx = -dy;
                    dy = dx_prev;
                }
                else
                {
                    dx = 0;
                    dy = 1;
                }

                // For all nodes surrounding the end of the end of the path nn
                for (uint i = 0; i < 4; ++i)
                {
                    // clang-format off
                    Point_t p2 = {  // NOLINT
                        nn.back().pos.x + dx,
                        nn.back().pos.y + dy,
                    };
                    // clang-format on

#ifdef DEBUG_PLOT
                    std::cout << "A*: Look around " << i << " at p2=(" << p2 << std::endl;
#endif

                    if (p2.x >= 0 && p2.x < nCols && p2.y >= 0 && p2.y < nRows)  // Bounds check, do not sep out of map
                    {
                        // If the new node (a neighbor of the end of the path nn) is open, append it to newPath ( = nn)
                        // and add that to the open1-list of paths.
                        // Because of the pop_back on open1, what happens is that the path is temporarily 'checked out',
                        // modified here, and then added back (if the condition above and below holds)
                        if (closed[p2.y][p2.x] == eNodeOpen && grid[p2.y][p2.x] == eNodeOpen)
                        {
#ifdef DEBUG_PLOT
                            std::cout << "A*: p2=" << p2 << " is OPEN" << std::endl;
#endif
                            std::vector<gridNode_t> newPath = nn;
                            // # heuristic  has to be designed to prefer a CCW turn
                            Point_t new_point = {p2.x, p2.y};
                            // clang-format off
                            gridNode_t new_node = {  // NOLINT
                                new_point,              // Point: x,y
                                cost + nn.back().cost,  // Cost
                                cost + nn.back().cost + distanceToClosestPoint(p2, open_space) + i,
                                // Heuristic (+i so CCW turns are cheaper)
                            };
                            // clang-format on

                            newPath.push_back(new_node);
                            closed[new_node.pos.y][new_node.pos.x] =
                                eNodeVisited;  // New node is now used in a path and thus visited

#ifdef DEBUG_PLOT
                            std::cout << "A*: Marked new_node " << new_node << " as eNodeVisited (true)" << std::endl;
                            std::cout << "A*: Add path from " << newPath.front().pos << " to " << newPath.back().pos
                                      << " of length " << newPath.size() << " to open1" << std::endl;
#endif
                            open1.push_back(newPath);
                        }
#ifdef DEBUG_PLOT
                        else
                        {
                            std::cout << "A*: p2=" << p2
                                      << " is not open: "
                                         "closed["
                                      << p2.y << "][" << p2.x << "]=" << closed[p2.y][p2.x]
                                      << ", "
                                         "grid["
                                      << p2.y << "][" << p2.x << "]=" << grid[p2.y][p2.x] << std::endl;
                        }
#endif
                    }
#ifdef DEBUG_PLOT
                    else
                    {
                        std::cout << "A*: p2=(" << p2.x << ", " << p2.y << ") is out of bounds" << std::endl;
                    }
#endif
                    // Cycle around to next neighbor, CCW
                    dx_prev = dx;
                    dx = dy;
                    dy = -dx_prev;
                }
            }
        }
    }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void printGrid(std::vector<std::vector<bool>> const& grid, std::vector<std::vector<bool>> const& visited,
               std::list<Point_t> const& path)
{
    for (uint iy = grid.size() - 1; iy >= 0; --iy)
    {
        for (uint ix = 0; ix < grid[0].size(); ++ix)
        {
            if (visited[iy][ix])
            {
                if (ix == path.front().x &&
                    iy == path.front().y)  // path.front = (starting) // whatis this kind of code????
                {
                    std::cout << "\033[1;32m▓\033[0m";  // Show starting position in green color
                }
                else if (ix == path.back().x && iy == path.back().y)  // path.back = red
                {
                    std::cout << "\033[1;31m▓\033[0m";  // Show stopping position in red color
                }
                else if (visited[iy][ix] && grid[iy][ix])
                {
                    std::cout << "\033[1;33m▓\033[0m";  // Show walls in yellow color
                }
                else
                {
                    std::cout << "\033[1;36m▓\033[0m";  // what is this again? unvisited?
                }
            }
            else
            {
                std::cout << "\033[1;37m▓\033[0m";
            }
        }
        std::cout << "\n";
    }
}

// ????????? why do we need multiple print grid
void printGrid(std::vector<std::vector<bool>> const& grid, std::vector<std::vector<bool>> const& visited,
               std::list<gridNode_t> const& path, gridNode_t start, gridNode_t end)
{
    for (uint iy = grid.size() - 1; iy >= 0; --iy)
    {
        for (uint ix = 0; ix < grid[0].size(); ++ix)
        {
            if (visited[iy][ix])
            {
                if (ix == start.pos.x && iy == start.pos.y)
                {
                    std::cout << "\033[1;32m▓\033[0m";  // Show starting position in green color
                }
                else if (ix == end.pos.x && iy == end.pos.y)
                {
                    std::cout << "\033[1;31m▓\033[0m";  // Show stopping position in red color
                }
                else if (visited[iy][ix] && grid[iy][ix])
                {
                    std::cout << "\033[1;33m▓\033[0m";  // Show walls in yellow color
                }
                else
                {
                    std::cout << "\033[1;36m▓\033[0m";
                }
            }
            else
            {
                std::cout << "\033[1;37m▓\033[0m";
            }
        }
        std::cout << "\n";
    }
}

void printGrid(std::vector<std::vector<bool>> const& grid)
{
    for (uint iy = grid.size() - 1; iy >= 0; --iy)
    {
        for (uint ix = 0; ix < grid[0].size(); ++ix)
        {
            if (grid[iy][ix])
            {
                std::cout << "\033[1;36m▓\033[0m";
            }
            else
            {
                std::cout << "\033[1;37m▓\033[0m";
            }
        }
        std::cout << "\n";
    }
}

std::list<Point_t> map_2_goals(std::vector<std::vector<bool>> const& grid, bool value_to_search)
{
    std::list<Point_t> goals;
    int ix, iy;
    uint nRows = grid.size();
    uint nCols = grid[0].size();
    for (iy = 0; iy < nRows; ++(iy))
    {
        for (ix = 0; ix < nCols; ++(ix))
        {
            if (grid[iy][ix] == value_to_search)

            //?????????? is the given value to search True or False?? what is the end purpose of this function?
            {
                Point_t p = {ix, iy};  // x, y
                goals.push_back(p);
                //????????????? what's the difference on push_back?
            }
        }
    }
    return goals;
}

void printPathNodes(std::list<gridNode_t> pathNodes)
{
    for (gridNode_t node : pathNodes)
    {
        std::cout << "(" << node.pos.x << ", " << node.pos.y << ")"
                  << ": " << node.cost << " " << node.he << std::endl;
    }
    std::cout << "--------------------------------------" << std::endl;
}

bool validMove(int x2, int y2, int nCols, int nRows, std::vector<std::vector<bool>> const& grid,
               std::vector<std::vector<bool>> const& visited)
{
    return (x2 >= 0 && x2 < nCols && y2 >= 0 && y2 < nRows)                 // path node is within the map
           && (grid[y2][x2] == eNodeOpen && visited[y2][x2] == eNodeOpen);  // the path node is unvisited
    // ??????????? meaning, not visited, and no obstacles.
}

void addNodeToList(int x2, int y2, std::list<gridNode_t>& pathNodes, std::vector<std::vector<bool>>& visited)
{
    Point_t new_point = {x2, y2};
    // clang-format off
    gridNode_t new_node = {  // NOLINT
        new_point,  // Point: x,y
        0,          // Cost
        0,          // Heuristic
    };
    // clang-format on
    pathNodes.push_back(
        new_node);  // turn point into gridnode and pushback in to path node to add new node!! ** add make it visited
    visited[y2][x2] = eNodeVisited;  // Close node
    return;
}

int dirWithMostSpace(int x_init, int y_init, int nCols, int nRows, std::vector<std::vector<bool>> const& grid,
                     std::vector<std::vector<bool>> const& visited, int ignoreDir)
{
    // this array stores how far the robot can travel in a straight line for each direction
    int free_space_in_dir[5] = {0};
    // for each direction
    for (int i = 1; i < 5; i++)
    {
        // start from starting pos
        int x2 = x_init;
        int y2 = y_init;
        do
        {  // loop until hit wall
            switch (i)
            {
            case east:
                x2++;
                break;
            case west:
                x2--;
                break;
            case north:
                y2++;
                break;
            case south:
                y2--;
                break;
            default:
                break;
            }
            free_space_in_dir[i]++;
            // counter for space
        } while (validMove(x2, y2, nCols, nRows, grid, visited));  // NOLINT
    }

    //????????? use the biggest value***->
    // set initial direction towards direction with most travel possible

    int indexValue = 0;
    int robot_dir = 1;
    for (int i = 1; i <= 4; i++)
    {
        // std::cout << "free space in " << i << ": " << free_space_in_dir[i] << std::endl;
        if (free_space_in_dir[i] > indexValue && i != ignoreDir)
        {
            robot_dir = i;
            indexValue = free_space_in_dir[i];
        }
    }
    return robot_dir;
}
