//
// Copyright [2020] Nobleo Technology"  [legal/copyright]
//
#include "full_coverage_path_planner/full_coverage_path_planner.h"

#include <list>
#include <vector>

/*  *** Note the coordinate system ***
 *  grid[][] is a 2D-vector:
 *  ***where ix is column-index and x-coordinate in map,
 *  iy is row-index and y-coordinate in map.
 *
 *            Cols  [ix]
 *        _______________________
 *       |__|__|__|__|__|__|__|__|
 *       |__|__|__|__|__|__|__|__|
 * Rows  |__|__|__|__|__|__|__|__|
 * [iy]  |__|__|__|__|__|__|__|__|
 *       |__|__|__|__|__|__|__|__|
 *y-axis |__|__|__|__|__|__|__|__|
 *   ^   |__|__|__|__|__|__|__|__|
 *   ^   |__|__|__|__|__|__|__|__|
 *   |   |__|__|__|__|__|__|__|__|
 *   |   |__|__|__|__|__|__|__|__|
 *
 *   O   --->> x-axis
 */

// #define DEBUG_PLOT

// Default Constructor
namespace full_coverage_path_planner
{
FullCoveragePathPlanner::FullCoveragePathPlanner()
    : initialized_(false)
{
}

void FullCoveragePathPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path)
{
    if (!initialized_)
    {
        ROS_ERROR(
            "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    // create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    //???????????? why do you need to create gui_path AND resize the path size? (to have same size as path.size?)

    if (!path.empty())
    {
        gui_path.header.frame_id = path[0].header.frame_id;
        gui_path.header.stamp = path[0].header.stamp;
    }

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame

    // ????????are moving a "local" plan to the world' coordinate frame here?? how does the publishing process work?
    for (unsigned int i = 0; i < path.size(); i++)
    {
        gui_path.poses[i] = path[i];
    }

    plan_pub_.publish(gui_path);
    //?????????? where does plan_pub_ come from?
}

void FullCoveragePathPlanner::parsePointlist2Plan(const geometry_msgs::PoseStamped& start,
                                                  std::list<Point_t> const& goalpoints,
                                                  std::vector<geometry_msgs::PoseStamped>& plan)
{
    //????????? how does this work???
    geometry_msgs::PoseStamped new_goal;
    std::list<Point_t>::const_iterator it, it_next, it_prev;
    int dx_now, dy_now, dx_next, dy_next, move_dir_now = 0, move_dir_prev = 0, move_dir_next = 0;
    bool do_publish = false;
    float orientation = eDirNone;
    ROS_INFO("Received goalpoints with length: %lu", goalpoints.size());
    if (goalpoints.size() > 1)
    {
        for (it = goalpoints.begin(); it != goalpoints.end(); ++it)
        {
            //??????????? how does this iterator work???
            it_next = it;
            it_next++;
            it_prev = it;
            it_prev--;

            // Check for the direction of movement
            if (it == goalpoints.begin())
            {
                dx_now = it_next->x - it->x;
                dy_now = it_next->y - it->y;
            }
            else
            {
                dx_now = it->x - it_prev->x;  //??????????? whats dx now???? (are we trying to get two directions???)
                dy_now = it->y - it_prev->y;
                dx_next = it_next->x - it->x;
                dy_next = it_next->y - it->y;
            }

            // Calculate direction enum: dx + dy*2 will give a unique number for each of the four possible directions
            // because of their signs:
            //  1 +  0*2 =  1
            //  0 +  1*2 =  2
            // -1 +  0*2 = -1
            //  0 + -1*2 = -2
            /*  eDirNone = 0,
        eDirRight = 1,
        eDirUp = 2,
        eDirLeft = -1,
        eDirDown = -2,*/
            move_dir_now = dx_now + dy_now * 2;
            move_dir_next = dx_next + dy_next * 2;

            // Check if this points needs to be published (i.e. a change of direction or first or last point in list)
            do_publish = move_dir_next != move_dir_now || it == goalpoints.begin() ||
                         (it != goalpoints.end() && it == --goalpoints.end());
            move_dir_prev = move_dir_now;

            //????????????? publish means to get robot to turn? (do_publish) means it was successful?

            // Add to vector if required
            if (do_publish)
            {
                new_goal.header.frame_id = "map";
                //?????????????? why tile size * 0.5???
                new_goal.pose.position.x = (it->x) * tile_size_ + grid_origin_.x + tile_size_ * 0.5;
                new_goal.pose.position.y = (it->y) * tile_size_ + grid_origin_.y + tile_size_ * 0.5;
                //?????????????? how did we get the above equations??
                // Calculate desired orientation to be in line with movement direction
                switch (move_dir_now)
                {
                case eDirNone:
                    // Keep orientation
                    //???????????????? how does orientation work below?????????
                    break;
                case eDirRight:
                    orientation = 0;
                    break;
                case eDirUp:
                    orientation = M_PI / 2;
                    break;
                case eDirLeft:
                    orientation = M_PI;
                    break;
                case eDirDown:
                    orientation = M_PI * 1.5;
                    break;
                }
                new_goal.pose.orientation = tf::createQuaternionMsgFromYaw(orientation);
                // ???????????? is this getting info from IMU??
                if (it != goalpoints.begin())
                {
                    // ?????????? how do we know it has changed direction?
                    previous_goal_.pose.orientation = new_goal.pose.orientation;
                    // ?????????????why do we republish previous goal (with current direction?)
                    // republish previous goal but with new orientation to indicate change of direction
                    // ???????? useful when the plan is strictly followed with base_link
                    plan.push_back(previous_goal_);
                }
                ROS_DEBUG("Voila new point: x=%f, y=%f, o=%f,%f,%f,%f", new_goal.pose.position.x,
                          new_goal.pose.position.y, new_goal.pose.orientation.x, new_goal.pose.orientation.y,
                          new_goal.pose.orientation.z, new_goal.pose.orientation.w);
                plan.push_back(new_goal);
                previous_goal_ = new_goal;
            }
        }
    }
    else
    {
        // ??????????? when does this else statement get triggered?
        new_goal.header.frame_id = "map";
        new_goal.pose.position.x = (goalpoints.begin()->x) * tile_size_ + grid_origin_.x + tile_size_ * 0.5;
        new_goal.pose.position.y = (goalpoints.begin()->y) * tile_size_ + grid_origin_.y + tile_size_ * 0.5;
        new_goal.pose.orientation = tf::createQuaternionMsgFromYaw(0);
        plan.push_back(new_goal);
    }
    /* Add poses from current position to start of plan */
    //???????????? what does it mean to add poses from current position to start of plan***
    // Compute angle between current pose and first plan point
    //?????????? is first plan point, the first point on the path?
    double dy = plan.begin()->pose.position.y - start.pose.position.y;
    double dx = plan.begin()->pose.position.x - start.pose.position.x;
    // Arbitrary choice of 100.0*FLT_EPSILON to determine minimum angle precision of 1%
    if (!(fabs(dy) < 100.0 * FLT_EPSILON && fabs(dx) < 100.0 * FLT_EPSILON))
    {
        //???????????? how is this used?? (when is the condition true?)
        //    the difference between 1 and the least value greater than 1 that is representable in the given floating
        //    point type b^(1−p)
        // Returns the absolute value of x: |x|.

        //??????????? how does this work?
        // Add extra translation waypoint
        double yaw = std::atan2(dy, dx);
        geometry_msgs::Quaternion quat_temp = tf::createQuaternionMsgFromYaw(yaw);
        // ????????? why add add extra pose?
        geometry_msgs::PoseStamped extra_pose;
        extra_pose = *plan.begin();
        //???????? why is this the only pointer???
        extra_pose.pose.orientation = quat_temp;
        plan.insert(plan.begin(), extra_pose);
        extra_pose = start;
        extra_pose.pose.orientation = quat_temp;
        //?????????????? why extra_pose.porientation = quant temp twice????????
        plan.insert(plan.begin(), extra_pose);
    }

    // Insert current pose
    plan.insert(plan.begin(), start);

    ROS_INFO("Plan ready containing %lu goals!", plan.size());
}

bool FullCoveragePathPlanner::parseCostmap(costmap_2d::Costmap2D* costmap_grid_, std::vector<std::vector<bool>>& grid,
                                           float robotRadius, float toolRadius,
                                           geometry_msgs::PoseStamped const& realStart, Point_t& scaledStart)
{
    int ix, iy, nodeRow, nodeCol;
    uint32_t nodeSize = dmax(floor(toolRadius / costmap_grid_->getResolution()), 1);  // Size of node in pixels/units
    //???????????? can you help me understand the diference betwene -> and also what does the "node" size mean and is
    // get resolution 30cm?
    //???????????? also, what is tool radius?
    uint32_t nRows = costmap_grid_->getSizeInCellsY(), nCols = costmap_grid_->getSizeInCellsX();
    ROS_INFO("nRows: %u nCols: %u nodeSize: %d", nRows, nCols, nodeSize);

    if (nRows == 0 || nCols == 0)
    {
        return false;
    }

    // Save map origin and scaling
    //????????? help to diffrentiate which one is tile, which one is node and which one is getresolution?????
    tile_size_ = nodeSize * costmap_grid_->getResolution();  // Size of a tile in meters
    grid_origin_.x = costmap_grid_->getOriginX();            // x-origin in meters
    grid_origin_.y = costmap_grid_->getOriginY();            // y-origin in meters
    ROS_INFO("costmap resolution: %g", costmap_grid_->getResolution());
    ROS_INFO("tile size: %g", tile_size_);
    ROS_INFO("grid origin: (%g, %g)", grid_origin_.x, grid_origin_.y);
    //????????? is grid origin datum????

    // Scale starting point
    scaledStart.x = static_cast<unsigned int>(
        clamp_((realStart.pose.position.x - grid_origin_.x) / tile_size_, 0.0, floor(nCols / tile_size_)));

    scaledStart.y = static_cast<unsigned int>(
        clamp_((realStart.pose.position.y - grid_origin_.y) / tile_size_, 0.0, floor(nRows / tile_size_)));

    ROS_INFO("real start: (%g, %g)", realStart.pose.position.x, realStart.pose.position.y);
    ROS_INFO("scaled start: (%u, %u)", scaledStart.x, scaledStart.y);

    // Scale grid
    for (iy = 0; iy < nRows; iy = iy + nodeSize)
    {
        std::vector<bool> gridRow;
        for (ix = 0; ix < nCols; ix = ix + nodeSize)
        {
            //?????????? where do we specify that above 65 is occupied??
            bool nodeOccupied = false;
            for (nodeRow = 0; (nodeRow < nodeSize) && ((iy + nodeRow) < nRows) && (nodeOccupied == false); ++nodeRow)
            {
                //???????????? what does the conditions mean??????   (nodeRow < nodeSize) && ((iy + nodeRow) < nRows)
                for (nodeCol = 0; (nodeCol < nodeSize) && ((ix + nodeCol) < nCols); ++nodeCol)
                {
                    double mx = ix + nodeCol;
                    double my = iy + nodeRow;
                    if (costmap_grid_->getCost(mx, my) > costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
                    {
                        //?????????? how does this if statement become true?
                        nodeOccupied = true;
                        // ROS_INFO("(%f, %f) marked occupied", mx, my);
                        break;
                    }
                }
            }
            gridRow.push_back(nodeOccupied);  // what does this push_back mean??
        }
        grid.push_back(gridRow);
    }
    return true;
}

// ???????????? can you help me understand why we have a parse grid and a parse costmap function???

bool FullCoveragePathPlanner::parseGrid(nav_msgs::OccupancyGrid const& cpp_grid_, std::vector<std::vector<bool>>& grid,
                                        float robotRadius, float toolRadius,
                                        geometry_msgs::PoseStamped const& realStart, Point_t& scaledStart)
{

#if (0)
    ROS_INFO(" ********************cpp_grid_.info.resolution = %f  toolRadius = %f ",cpp_grid_.info.resolution, toolRadius);
#endif

    int ix, iy, nodeRow, nodeColl;
    uint32_t nodeSize = dmax(floor(toolRadius / cpp_grid_.info.resolution), 1);        // Size of node in pixels/units  5
    uint32_t robotNodeSize = dmax(floor(robotRadius / cpp_grid_.info.resolution), 1);  // RobotRadius in pixels/units   5
    // ?????????????? what is robot nodesize????
    uint32_t nRows = cpp_grid_.info.height, nCols = cpp_grid_.info.width;
    ROS_INFO("nRows: %u nCols: %u nodeSize: %d", nRows, nCols, nodeSize);

    if (nRows == 0 || nCols == 0)
    {
        return false;
    }

    // Save map origin and scaling
    tile_size_ = nodeSize * cpp_grid_.info.resolution;  // Size of a tile in meters   0.25 m

    // 找到了小样-----这就是origin的使用地方
    grid_origin_.x = cpp_grid_.info.origin.position.x;  // x-origin in meters       
    grid_origin_.y = cpp_grid_.info.origin.position.y;  // y-origin in meters    
    ROS_INFO("costmap resolution: %g", cpp_grid_.info.resolution);
    ROS_INFO("tile size: %g", tile_size_);
    ROS_INFO("grid origin: (%g, %g)", grid_origin_.x, grid_origin_.y);

    // Scale starting point
    scaledStart.x = static_cast<unsigned int>(clamp_((realStart.pose.position.x - grid_origin_.x) / tile_size_, 0.0,
                                                     floor(cpp_grid_.info.width / tile_size_)));
    scaledStart.y = static_cast<unsigned int>(clamp_((realStart.pose.position.y - grid_origin_.y) / tile_size_, 0.0,
                                                     floor(cpp_grid_.info.height / tile_size_)));
    ROS_INFO("real start: (%g, %g)", realStart.pose.position.x, realStart.pose.position.y);
    ROS_INFO("scaled start: (%u, %u)", scaledStart.x, scaledStart.y);

    // Scale grid 
    // nodeSize 就是小车宽度  每次跳过的时候得跳过车车的宽度啊
    for (iy = 0; iy < nRows; iy = iy + nodeSize)
    {
        std::vector<bool> gridRow;
        for (ix = 0; ix < nCols; ix = ix + nodeSize)
        {
            bool nodeOccupied = false;

            // ????????? how come scale grid has a nodeOccupied too? (snow dump area is here?)
            // 这里是进行障碍处理的
            for (nodeRow = 0; (nodeRow < robotNodeSize) && ((iy + nodeRow) < nRows) && (nodeOccupied == false);   
                 ++nodeRow)
            {
                //??????????? why robotnodesize???
                for (nodeColl = 0; (nodeColl < robotNodeSize) && ((ix + nodeColl) < nCols); ++nodeColl)  
                {
                    int index_grid =
                        dmax((iy + nodeRow - ceil(static_cast<float>(robotNodeSize - nodeSize) / 2.0)) * nCols +
                                 (ix + nodeColl - ceil(static_cast<float>(robotNodeSize - nodeSize) / 2.0)),
                             0);
                    //?????????????? how does this work????
                    if (cpp_grid_.data[index_grid] > 65)
                    {
                        //??????????????????? how does this number get calculated????
                        nodeOccupied = true;
                        break;
                    }
                }
            }
            gridRow.push_back(nodeOccupied);
        }
        grid.push_back(gridRow);
    }
    return true;
}
}  // namespace full_coverage_path_planner
