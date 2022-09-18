/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include "a_star_planner.h"

namespace roborts_global_planner{

    using roborts_common::ErrorCode;
    using roborts_common::ErrorInfo;
    //构造函数,地图宽高初始化
    AStarPlanner::AStarPlanner(CostmapPtr costmap_ptr) :
            GlobalPlannerBase::GlobalPlannerBase(costmap_ptr),
            gridmap_width_(costmap_ptr_->GetCostMap()->GetSizeXCell()),
            gridmap_height_(costmap_ptr_->GetCostMap()->GetSizeYCell()),
            cost_(costmap_ptr_->GetCostMap()->GetCharMap()) {

        AStarPlannerConfig a_star_planner_config;
        //参数文件获取
        std::string full_path = ros::package::getPath("roborts_planning") + "/global_planner/a_star_planner/"\
      "config/a_star_planner_config.prototxt";
        //如果没有读取到文件,则打印error信息
        if (!roborts_common::ReadProtoFromTextFile(full_path.c_str(),
                                                   &a_star_planner_config)) {
            ROS_ERROR("Cannot load a star planner protobuf configuration file.");
        }
        // A*规划参数配置
        heuristic_factor_ = a_star_planner_config.heuristic_factor();
        inaccessible_cost_ = a_star_planner_config.inaccessible_cost();
        goal_search_tolerance_ = a_star_planner_config.goal_search_tolerance()/costmap_ptr->GetCostMap()->GetResolution();
        beziercruve_pointsize_ = a_star_planner_config.beziercruve_pointsize();
    }
    //析构函数,将栅格地图数据清除
    AStarPlanner::~AStarPlanner(){
        cost_ = nullptr;
    }
    /**
     * @brief A星算法实现主要函数
     * 
     * @param start 起始点位姿
     * @param goal 目标点位姿
     * @param path 容器储存了构成路径的一系列位姿点
     * @return ErrorInfo 
     */
    ErrorInfo AStarPlanner::Plan(const geometry_msgs::PoseStamped &start,
                                 const geometry_msgs::PoseStamped &goal,
                                 std::vector<geometry_msgs::PoseStamped> &path) {
        //起始点,临时点的x/y定义
        unsigned int start_x, start_y, goal_x, goal_y, tmp_goal_x, tmp_goal_y;
        //存储目标点的x,y数组
        unsigned int valid_goal[2];
        // 最短的距离
        unsigned  int shortest_dist = std::numeric_limits<unsigned int>::max();
        //标志位,目标是否有效
        bool goal_valid = false;
        //如果没有将起始姿势从地图框架转换为代价地图框架,则打印warning信息
        if (!costmap_ptr_->GetCostMap()->World2Map(start.pose.position.x,
                                                   start.pose.position.y,
                                                   start_x,
                                                   start_y)) {
            ROS_WARN("Failed to transform start pose from map frame to costmap frame");
            return ErrorInfo(ErrorCode::GP_POSE_TRANSFORM_ERROR,
                             "Start pose can't be transformed to costmap frame.");
        }
        //如果没有将起始姿势从地图框架转换为代价地图框架,则打印warning信息
        if (!costmap_ptr_->GetCostMap()->World2Map(goal.pose.position.x,
                                                   goal.pose.position.y,
                                                   goal_x,
                                                   goal_y)) {
            ROS_WARN("Failed to transform goal pose from map frame to costmap frame");
            return ErrorInfo(ErrorCode::GP_POSE_TRANSFORM_ERROR,
                             "Goal pose can't be transformed to costmap frame.");
        }
        //如果获取的目标点低于访问成本(目标点可到达),则将该目标点x,y存到有效目标数组上
        if (costmap_ptr_->GetCostMap()->GetCost(goal_x,goal_y)<inaccessible_cost_){
            valid_goal[0] = goal_x;
            valid_goal[1] = goal_y;
            goal_valid = true; //标志位变为true
        }
        //目标点在障碍物话就存到临时目标点,如果该点在goal_search_tolerance_内,则将该点放进valid_goal[2]数组里
        else{
            tmp_goal_x = goal_x;
            tmp_goal_y = goal_y - goal_search_tolerance_;
            //当临时目标点的y值<=目标值y点值+目标容忍度值,则执行while
            while(tmp_goal_y <= goal_y + goal_search_tolerance_){
                tmp_goal_x = goal_x - goal_search_tolerance_;
                //同上
                while(tmp_goal_x <= goal_x + goal_search_tolerance_){
                    //计算代价值
                    unsigned char cost = costmap_ptr_->GetCostMap()->GetCost(tmp_goal_x, tmp_goal_y);
                    //计算距离值
                    unsigned int dist = abs(static_cast<int>(goal_x - tmp_goal_x)) + abs(static_cast<int>(goal_y - tmp_goal_y));
                    //如果代价小于成本,距离小于最短距离,则将临时目标点置为有效目标点
                    if (cost < inaccessible_cost_ && dist < shortest_dist ) {
                        shortest_dist = dist;
                        valid_goal[0] = tmp_goal_x;
                        valid_goal[1] = tmp_goal_y;
                        goal_valid = true; //标志位变为true
                    }
                    tmp_goal_x += 1;
                }
                tmp_goal_y += 1;
            }
        }
        ErrorInfo error_info;
        //标志位为false,找不到目标点,清除生成的路径
        if (!goal_valid){
            error_info=ErrorInfo(ErrorCode::GP_GOAL_INVALID_ERROR);
            path.clear();
        }
        //有目标点
        else{
            unsigned int start_index, goal_index;
            start_index = costmap_ptr_->GetCostMap()->GetIndex(start_x, start_y);
            goal_index = costmap_ptr_->GetCostMap()->GetIndex(valid_goal[0], valid_goal[1]);

            costmap_ptr_->GetCostMap()->SetCost(start_x, start_y,roborts_costmap::FREE_SPACE);
            //如果起始点的index==目标值的index
            if(start_index == goal_index){
                error_info=ErrorInfo::OK();
                path.clear();   //清除路径
                path.push_back(start);  //将起始点push进path
                path.push_back(goal);   //将目标点push进path
            }
            //如果起始点的index != 目标值的index,则寻找路径
            else{
                //执行SearchPath函数
                error_info = SearchPath(start_index, goal_index, path);
                //如果找到路径了,则进行贝路径平滑
                if ( error_info.IsOK() ){
                    geometry_msgs::PoseStamped path_goal = path.back();//back用于访问向量的最后一个元素，返回对向量最后一个元素的引用。
                    std::vector<geometry_msgs::PoseStamped> finish_path;
                    finish_path.assign(path.begin(),path.end());
                    for(int i = 0 ; i < 2 ; i++){
                        PathCurve(finish_path);
                    }
                    if(finish_path.size() > 0){
                        path.assign(finish_path.begin(),finish_path.end());
                        path.push_back(path_goal);
                    }
                    path.back().pose.orientation = goal.pose.orientation;
                    path.back().pose.position.z = goal.pose.position.z;
                }
            }

        }


        return error_info;
    }
    /**
     * @brief 寻找路径
     * 
     * @param start_index 起始点的index
     * @param goal_index 目标点的index
     * @param path 容器储存了构成路径的一系列位姿点
     * @return ErrorInfo 
     */
    ErrorInfo AStarPlanner::SearchPath(const int &start_index,
                                       const int &goal_index,
                                       std::vector<geometry_msgs::PoseStamped> &path) {
        //将vector清空
        g_score_.clear();
        f_score_.clear();
        parent_.clear();
        state_.clear();
        //获取地图的x单元格和y单元格大小
        gridmap_width_ = costmap_ptr_->GetCostMap()->GetSizeXCell();
        gridmap_height_ = costmap_ptr_->GetCostMap()->GetSizeYCell();
        ROS_INFO("Search in a map %d", gridmap_width_*gridmap_height_);
        cost_ = costmap_ptr_->GetCostMap()->GetCharMap();
        g_score_.resize(gridmap_height_ * gridmap_width_, std::numeric_limits<int>::max());
        f_score_.resize(gridmap_height_ * gridmap_width_, std::numeric_limits<int>::max());
        parent_.resize(gridmap_height_ * gridmap_width_, std::numeric_limits<int>::max());
        state_.resize(gridmap_height_ * gridmap_width_, SearchState::NOT_HANDLED);
        //定义open列表
        std::priority_queue<int, std::vector<int>, Compare> openlist;
        g_score_.at(start_index) = 0;
        openlist.push(start_index);  //将起始点加进open列表

        std::vector<int> neighbors_index;
        int current_index, move_cost, h_score, count = 0;

        //执行循环,直至open列表为空
        while (!openlist.empty()) {
            //将当前的index值设为open列表的第一位
            current_index = openlist.top();
            //删除open列表的最后一位
            openlist.pop();
            //将当前的index值加进close列表
            state_.at(current_index) = SearchState::CLOSED;
            //如果当前的index == 目标点的index,则跳出循环
            if (current_index == goal_index) {
                ROS_INFO("Search takes %d cycle counts", count);
                break;
            }
            //获取相邻的八个点
            GetNineNeighbors(current_index, neighbors_index);
            //循环相邻的八个点
            for (auto neighbor_index : neighbors_index) {
                //对相邻点的index非法值处理
                if (neighbor_index < 0 ||
                    neighbor_index >= gridmap_height_ * gridmap_width_) {
                    continue;
                }
                //如果相邻点的index成本太大(不可到达的相邻点,比如此点在障碍物里)
                if (cost_[neighbor_index] >= inaccessible_cost_ ||
                    state_.at(neighbor_index) == SearchState::CLOSED) {
                    continue;
                }
                //获取移动代价
                GetMoveCost(current_index, neighbor_index, move_cost);
                //如果在相邻点的G代价 > 在当前的index的G代价 + 移动到相邻点的移动代价 + 相邻点的index代价
                if (g_score_.at(neighbor_index) > g_score_.at(current_index) + move_cost + cost_[neighbor_index]) {

                    g_score_.at(neighbor_index) = g_score_.at(current_index) + move_cost + cost_[neighbor_index];
                    //将父节点设置为当前的index
                    parent_.at(neighbor_index) = current_index;
                    //如果相邻点的index状态为未处理的单元格
                    if (state_.at(neighbor_index) == SearchState::NOT_HANDLED) {
                        //h_score = 相邻点到目标点的曼哈顿距离
                        GetManhattanDistance(neighbor_index, goal_index, h_score);
                        //计算F值
                        f_score_.at(neighbor_index) = g_score_.at(neighbor_index) + h_score;
                        //将相邻点加进open列表
                        openlist.push(neighbor_index);
                        //将相邻点的状态设置为处在open队列中
                        state_.at(neighbor_index) = SearchState::OPEN;
                    }
                }
            }
            count++;
        }
        //如果当前的index不等于目标的index
        if (current_index != goal_index) {
            ROS_WARN("Global planner can't search the valid path!");
            return ErrorInfo(ErrorCode::GP_PATH_SEARCH_ERROR, "Valid global path not found.");
        }

        unsigned int iter_index = current_index, iter_x, iter_y;

        geometry_msgs::PoseStamped iter_pos;
        iter_pos.pose.orientation.w = 1;
        iter_pos.header.frame_id = "map";
        path.clear();
        costmap_ptr_->GetCostMap()->Index2Cells(iter_index, iter_x, iter_y);
        costmap_ptr_->GetCostMap()->Map2World(iter_x, iter_y, iter_pos.pose.position.x, iter_pos.pose.position.y);
        path.push_back(iter_pos);
        //找回路劲:除了起始方块, 每一个曾经或者现在还在 "开启列表" 里的方块, 它都有一个 "父方块", 通过 "父方块" 可以索引到最初的 "起始方块", 这就是路径。
        while (iter_index != start_index) {
            iter_index = parent_.at(iter_index);
//    if(cost_[iter_index]>= inaccessible_cost_){
//      LOG_INFO<<"Cost changes through planning for"<< static_cast<unsigned int>(cost_[iter_index]);
//    }
            costmap_ptr_->GetCostMap()->Index2Cells(iter_index, iter_x, iter_y);
            costmap_ptr_->GetCostMap()->Map2World(iter_x, iter_y, iter_pos.pose.position.x, iter_pos.pose.position.y);
            path.push_back(iter_pos);
        }

        std::reverse(path.begin(),path.end());

        return ErrorInfo(ErrorCode::OK);

    }
    /**
     * @brief 获取移动代价
     * 
     * @param current_index 当前的index
     * @param neighbor_index 相邻点的index
     * @param move_cost 移动的代价(10 or 14)
     * @return ErrorInfo 
     */
    ErrorInfo AStarPlanner::GetMoveCost(const int &current_index,
                                        const int &neighbor_index,
                                        int &move_cost) const {
        //上下左右四个点的移动代价为10
        if (abs(neighbor_index - current_index) == 1 ||
            abs(neighbor_index - current_index) == gridmap_width_) {
            move_cost = 10;
        } 
        //左上左下右上右下四个点的移动代价为14
        else if (abs(neighbor_index - current_index) == (gridmap_width_ + 1) ||
                   abs(neighbor_index - current_index) == (gridmap_width_ - 1)) {
            move_cost = 14;
        } else {
            return ErrorInfo(ErrorCode::GP_MOVE_COST_ERROR,
                             "Move cost can't be calculated cause current neighbor index is not accessible");
        }
        return ErrorInfo(ErrorCode::OK);
    }
    /**
     * @brief 获取曼哈顿距离
     * 
     * @param index1 
     * @param index2 
     * @param manhattan_distance 
     */
    void AStarPlanner::GetManhattanDistance(const int &index1, const int &index2, int &manhattan_distance) const {
        manhattan_distance = heuristic_factor_* 10 * (abs(static_cast<int>(index1 / gridmap_width_ - index2 / gridmap_width_)) +
                                                      abs(static_cast<int>((index1 % gridmap_width_ - index2 % gridmap_width_))));
    }
    /**
     * @brief 获取相邻的八个点
     * 
     * @param current_index 当前的index
     * @param neighbors_index 用来储存相邻八个点的一维vector
     */
    void AStarPlanner::GetNineNeighbors(const int &current_index, std::vector<int> &neighbors_index) const {
        neighbors_index.clear();
        if(current_index - gridmap_width_ >= 0){
            neighbors_index.push_back(current_index - gridmap_width_);       //up
        }
        if(current_index - gridmap_width_ - 1 >= 0 && (current_index - gridmap_width_ - 1 + 1) % gridmap_width_!= 0){
            neighbors_index.push_back(current_index - gridmap_width_ - 1); //left_up
        }
        if(current_index - 1 >= 0 && (current_index - 1 + 1) % gridmap_width_!= 0){
            neighbors_index.push_back(current_index - 1);        //left
        }
        if(current_index + gridmap_width_ - 1 < gridmap_width_* gridmap_height_
           && (current_index + gridmap_width_ - 1 + 1) % gridmap_width_!= 0){
            neighbors_index.push_back(current_index + gridmap_width_ - 1); //left_down
        }
        if(current_index + gridmap_width_ < gridmap_width_* gridmap_height_){
            neighbors_index.push_back(current_index + gridmap_width_);     //down
        }
        if(current_index + gridmap_width_ + 1 < gridmap_width_* gridmap_height_
           && (current_index + gridmap_width_ + 1 ) % gridmap_width_!= 0){
            neighbors_index.push_back(current_index + gridmap_width_ + 1); //right_down
        }
        if(current_index  + 1 < gridmap_width_* gridmap_height_
           && (current_index  + 1 ) % gridmap_width_!= 0) {
            neighbors_index.push_back(current_index + 1);                   //right
        }
        if(current_index - gridmap_width_ + 1 >= 0
           && (current_index - gridmap_width_ + 1 ) % gridmap_width_!= 0) {
            neighbors_index.push_back(current_index - gridmap_width_ + 1); //right_up
        }
    }
    /**
     * @brief 路劲平滑
     * 
     * @param path 
     */
    void AStarPlanner::PathCurve(std::vector<geometry_msgs::PoseStamped>& path){
        // temp path 用于截取路径的原始点用于平滑，平滑一次后将清空
        std::vector<geometry_msgs::PoseStamped> temp_path;
        // finish_path 保存 temp path 平滑后统一保存至 finish_path , 整条线段平滑后将线段复制到 path 中
        std::vector<geometry_msgs::PoseStamped> finish_path;
        for(unsigned int i = 0; i < path.size(); i+=beziercruve_pointsize_){
            temp_path.clear();
            for(unsigned int index = i ; index < i+beziercruve_pointsize_ ; index++){
                if(index >= path.size()){
                    break;
                }
                temp_path.push_back(path[index]);  //将已生成路径的点按beziercruve_pointsize_个点数来放进temp_path
            }
            std::vector<geometry_msgs::PoseStamped> temp_finish_path;
            //执行路径平滑函数
            temp_finish_path = BezierCurve(temp_path);
            finish_path.insert(finish_path.end(),temp_finish_path.begin(),temp_finish_path.end());
        }
        path.assign(finish_path.begin(),finish_path.end());
        finish_path.clear();
        temp_path.clear();
        finish_path.shrink_to_fit();//减少容器的容量以适应其大小并销毁超出容量的所有元素
        temp_path.shrink_to_fit();
    }
    /**
     * @brief 利用贝塞尔曲线原理优化路径
     * 
     * @param src_path 输入已生成的路径的beziercruve_pointsize_个点,依次平滑
     * @return std::vector<geometry_msgs::PoseStamped> 
     */
    std::vector<geometry_msgs::PoseStamped> AStarPlanner::BezierCurve(std::vector<geometry_msgs::PoseStamped> src_path)
    {
        if (src_path.size() < 1)//这种情况是不允许出现的，出现只能证明程序出错了
            return src_path;
        const float step = 0.25;//采集2个点，即1.0/step
        std::vector<geometry_msgs::PoseStamped> res;
        if (src_path.size() == 1) {//递归结束条件，k=0
            for (float t = 0; t < 1; t += step)
                res.push_back(src_path[0]);//为了和其他情况保持一致，生成了1.0/step个一样的点
            return res;
        }
        std::vector<geometry_msgs::PoseStamped> src1;
        std::vector<geometry_msgs::PoseStamped> src2;
        src1.assign(src_path.begin(), src_path.end() - 1);//分成两部分，即Pi和Pi+1
        src2.assign(src_path.begin() + 1, src_path.end());
        std::vector<geometry_msgs::PoseStamped> pln1 = BezierCurve(src1);
        std::vector<geometry_msgs::PoseStamped> pln2 = BezierCurve(src2);
        for (float t = 0; t < 1; t += step) {
            geometry_msgs::PoseStamped temp;
            temp.header.frame_id = "map";
            temp.pose.position.x = (1.0 - t) * pln1[(1.0 / step * t)].pose.position.x + t * pln2[(1.0 / step * t)].pose.position.x;
            temp.pose.position.y = (1.0 - t) * pln1[(1.0 / step * t)].pose.position.y + t * pln2[(1.0 / step * t)].pose.position.y;
            res.push_back(temp);
        }
        return res;
    }

    

} //namespace roborts_global_planner
