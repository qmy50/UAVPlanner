#include "path_searching/dyn_a_star_new.h"

using namespace std;
using namespace Eigen;

AStar::~AStar()
{
    for (int i = 0; i < POOL_SIZE_(0); i++)
        for (int j = 0; j < POOL_SIZE_(1); j++)
            for (int k = 0; k < POOL_SIZE_(2); k++)
                delete GridNodeMap_[i][j][k];
}

void AStar::initGridMap(GridMap::Ptr occ_map, const Eigen::Vector3i pool_size)
{
    POOL_SIZE_ = pool_size;
    CENTER_IDX_ = pool_size / 2;

    GridNodeMap_ = new GridNodePtr **[POOL_SIZE_(0)];
    for (int i = 0; i < POOL_SIZE_(0); i++)
    {
        GridNodeMap_[i] = new GridNodePtr *[POOL_SIZE_(1)];
        for (int j = 0; j < POOL_SIZE_(1); j++)
        {
            GridNodeMap_[i][j] = new GridNodePtr[POOL_SIZE_(2)];
            for (int k = 0; k < POOL_SIZE_(2); k++)
            {
                GridNodeMap_[i][j][k] = new GridNode;
            }
        }
    }

    grid_map_ = occ_map;
}

double AStar::getDiagHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    double h = 0.0;
    int diag = min(min(dx, dy), dz);
    dx -= diag;
    dy -= diag;
    dz -= diag;

    if (dx == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
    }
    if (dy == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
    }
    if (dz == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
    }
    return h;
}

double AStar::getManhHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    return dx + dy + dz;
}

double AStar::getEuclHeu(GridNodePtr node1, GridNodePtr node2)
{
    return (node2->index - node1->index).norm();
}

vector<GridNodePtr> AStar::retrievePath(GridNodePtr current)
{
    vector<GridNodePtr> path;
    path.push_back(current);

    while (current->cameFrom != NULL)
    {
        current = current->cameFrom;
        path.push_back(current);
    }

    return path;
}

bool AStar::ConvertToIndexAndAdjustStartEndPoints(Vector3d start_pt, Vector3d end_pt, Vector3i &start_idx, Vector3i &end_idx)
{
    if (!Coord2Index(start_pt, start_idx) || !Coord2Index(end_pt, end_idx))
        return false;

    int occ;
    if (checkOccupancy(Index2Coord(start_idx),drone_id_))
    {
        // ROS_WARN("Start point is insdide an obstacle.");
        do
        {
            start_pt = (start_pt - end_pt).normalized() * step_size_ + start_pt;
            // cout << "start_pt=" << start_pt.transpose() << endl;
            if (!Coord2Index(start_pt, start_idx))
            {
                return false;
            }

            occ = checkOccupancy(Index2Coord(start_idx),drone_id_);
            if (occ == -1)
            {
                Eigen::Vector3d pos = Index2Coord(start_idx);
                ROS_INFO("the start pos is x=%f,y=%f,z=%f",pos(0),pos(1),pos(2));
                ROS_WARN("[Astar] Start point outside the map region.");
                return false;
            }
        } while (occ);
    }

    if (checkOccupancy(Index2Coord(end_idx),drone_id_))
    {
        // ROS_WARN("End point is insdide an obstacle.");
        do
        {
            end_pt = (end_pt - start_pt).normalized() * step_size_ + end_pt;
            // cout << "end_pt=" << end_pt.transpose() << endl;
            if (!Coord2Index(end_pt, end_idx))
            {
                return false;
            }

            occ = checkOccupancy(Index2Coord(start_idx),drone_id_);
            if (occ == -1)
            {
                ROS_WARN("[Astar] End point outside the map region.");
                return false;
            }
        } while (checkOccupancy(Index2Coord(end_idx),drone_id_));
    }

    return true;
}

ASTAR_RET AStar::AstarSearch(const double step_size, Vector3d start_pt, Vector3d end_pt)
{
    ros::Time time_1 = ros::Time::now();
    ++rounds_;

    step_size_ = step_size;
    inv_step_size_ = 1 / step_size;
    center_ = (start_pt + end_pt) / 2;

    Vector3i start_idx, end_idx;
    if (!ConvertToIndexAndAdjustStartEndPoints(start_pt, end_pt, start_idx, end_idx))
    {
        ROS_ERROR("Unable to handle the initial or end point, force return!");
        return ASTAR_RET::INIT_ERR;
    }

    // if ( start_pt(0) > -1 && start_pt(0) < 0 )
    //     cout << "start_pt=" << start_pt.transpose() << " end_pt=" << end_pt.transpose() << endl;

    GridNodePtr startPtr = GridNodeMap_[start_idx(0)][start_idx(1)][start_idx(2)];
    GridNodePtr endPtr = GridNodeMap_[end_idx(0)][end_idx(1)][end_idx(2)];

    std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> empty;
    openSet_.swap(empty);

    GridNodePtr neighborPtr = NULL;
    GridNodePtr current = NULL;

    endPtr->index = end_idx;

    startPtr->index = start_idx;
    startPtr->rounds = rounds_;
    startPtr->gScore = 0;
    startPtr->fScore = getHeu(startPtr, endPtr);
    startPtr->state = GridNode::OPENSET; //put start node in open set
    startPtr->cameFrom = NULL;
    openSet_.push(startPtr); //put start in open set

    double tentative_gScore;

    int num_iter = 0;
    while (!openSet_.empty())
    {
        num_iter++;
        current = openSet_.top();
        openSet_.pop();

        // if ( num_iter < 10000 )
        //     cout << "current=" << current->index.transpose() << endl;

        if (current->index(0) == endPtr->index(0) && current->index(1) == endPtr->index(1) && current->index(2) == endPtr->index(2))
        {
            // ros::Time time_2 = ros::Time::now();
            // printf("\033[34mA star iter:%d, time:%.3f\033[0m\n",num_iter, (time_2 - time_1).toSec()*1000);
            // if((time_2 - time_1).toSec() > 0.1)
            //     ROS_WARN("Time consume in A star path finding is %f", (time_2 - time_1).toSec() );
            gridPath_ = retrievePath(current);
            return ASTAR_RET::SUCCESS;
        }
        current->state = GridNode::CLOSEDSET; //move current node from open set to closed set.

        for (int dx = -1; dx <= 1; dx++)
            for (int dy = -1; dy <= 1; dy++)
                for (int dz = -1; dz <= 1; dz++)
                {
                    if (dx == 0 && dy == 0 && dz == 0)
                        continue;

                    Vector3i neighborIdx;
                    neighborIdx(0) = (current->index)(0) + dx;
                    neighborIdx(1) = (current->index)(1) + dy;
                    neighborIdx(2) = (current->index)(2) + dz;

                    if (neighborIdx(0) < 1 || neighborIdx(0) >= POOL_SIZE_(0) - 1 || neighborIdx(1) < 1 || neighborIdx(1) >= POOL_SIZE_(1) - 1 || neighborIdx(2) < 1 || neighborIdx(2) >= POOL_SIZE_(2) - 1)
                    {
                        continue;
                    }

                    neighborPtr = GridNodeMap_[neighborIdx(0)][neighborIdx(1)][neighborIdx(2)];
                    neighborPtr->index = neighborIdx;

                    bool flag_explored = neighborPtr->rounds == rounds_;

                    if (flag_explored && neighborPtr->state == GridNode::CLOSEDSET)
                    {
                        continue; //in closed set.
                    }

                    neighborPtr->rounds = rounds_;

                    if (checkOccupancy(Index2Coord(neighborPtr->index),drone_id_))
                    {
                        continue;
                    }

                    double static_cost = sqrt(dx * dx + dy * dy + dz * dz);
                    tentative_gScore = current->gScore + static_cost;

                    if (!flag_explored)
                    {
                        //discover a new node
                        neighborPtr->state = GridNode::OPENSET;
                        neighborPtr->cameFrom = current;
                        neighborPtr->gScore = tentative_gScore;
                        neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                        openSet_.push(neighborPtr); //put neighbor in open set and record it.
                    }
                    else if (tentative_gScore < neighborPtr->gScore)
                    { //in open set and need update
                        neighborPtr->cameFrom = current;
                        neighborPtr->gScore = tentative_gScore;
                        neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                    }
                }
        ros::Time time_2 = ros::Time::now();
        if ((time_2 - time_1).toSec() > 0.2)
        {
            ROS_WARN("Failed in A star path searching !!! 0.2 seconds time limit exceeded.");
            return ASTAR_RET::SEARCH_ERR;
        }
    }

    ros::Time time_2 = ros::Time::now();

    if ((time_2 - time_1).toSec() > 0.1)
        ROS_WARN("Time consume in A star path finding is %.3fs, iter=%d", (time_2 - time_1).toSec(), num_iter);

    return ASTAR_RET::SEARCH_ERR;
}

bool AStar::AstarSearchWaypoints(const double step_size, 
                                 const std::vector<Eigen::Vector3d>& waypoints,
                                 std::vector<Eigen::Vector3d>& full_path)
{
    if (waypoints.size() < 2)
    {
        ROS_ERROR("[Astar] At least two waypoints (start and end) are required.");
        return false;
    }

    full_path.clear();

    // 依次规划每一段航点之间的路径
    for (size_t i = 0; i < waypoints.size() - 1; ++i)
    {
        const Eigen::Vector3d& start_pt = waypoints[i];
        const Eigen::Vector3d& end_pt   = waypoints[i+1];

        // 调用现有的 A* 单段规划函数
        ASTAR_RET ret = AstarSearch(step_size, start_pt, end_pt);

        if (ret != ASTAR_RET::SUCCESS)
        {
            ROS_ERROR("[Astar] Failed to plan segment from (%f, %f, %f) to (%f, %f, %f)",
                      start_pt.x(), start_pt.y(), start_pt.z(),
                      end_pt.x(),   end_pt.y(),   end_pt.z());
            return false;
        }

        // 获取当前段的路径（已按从起点到终点的顺序排列）
        std::vector<Eigen::Vector3d> segment_path = getPath();

        if (segment_path.empty())
        {
            ROS_ERROR("[Astar] Retrieved path is empty for segment %zu.", i);
            return false;
        }

        // 拼接路径：第一段包含起点，后续段去掉起点（避免重复点）
        if (i == 0)
        {
            full_path = segment_path;                // 第一段完整加入
        }
        else
        {
            // 后续段：跳过第一个点（即上一段的终点，已存在）
            full_path.insert(full_path.end(),
                             segment_path.begin() + 1,
                             segment_path.end());
        }
    }

    return true;
}

double AStar::pointToLineDistance(const Eigen::Vector3d& pt,const Eigen::Vector3d& line_start,const Vector3d& line_end)
{
    Eigen::Vector3d line = line_end - line_start;
    Eigen::Vector3d v = pt - line_start;
    double line_len = line.norm();

    if (line_len < 1e-6)
        return v.norm();

    double t = v.dot(line) / (line_len * line_len);
    t = max(0.0, min(1.0, t));

    Eigen::Vector3d proj = line_start + t * line;
    return (pt - proj).norm();
}

void AStar::rdpSimplify(vector<Eigen::Vector3d>& points, vector<Eigen::Vector3d>& result, double epsilon, int start, int end)
{
    if (start >= end)
        return;

    double max_dist = 0.0;
    int max_index = start;

    for (int i = start + 1; i < end; ++i)
    {
        double dist = pointToLineDistance(points[i], points[start], points[end]);
        if (dist > max_dist)
        {
            max_dist = dist;
            max_index = i;
        }
    }

    if (max_dist > epsilon)
    {
        rdpSimplify(points, result, epsilon, start, max_index);
        result.push_back(points[max_index]);
        rdpSimplify(points, result, epsilon, max_index, end);
    }
}

vector<Vector3d> AStar::getPath()
{
    vector<Vector3d> path;

    for (auto ptr : gridPath_)
        path.push_back(Index2Coord(ptr->index));

    reverse(path.begin(), path.end());
    // double rdp_epsilon = 0.05;
    // vector<Vector3d> simplified_path;
    // rdpSimplify(path,simplified_path,rdp_epsilon,0,path.size()-1);
    // ROS_WARN("The path size is %d",simplified_path.size());
    return path;
}
