#include "lidar_localization/path_planning/RRT.hpp"
#include <unistd.h>

namespace lidar_localization
{

    void RRT::init(const RRT::node &startPoint_, const RRT::node &endPoint_, const nav_msgs::OccupancyGrid &grid_map)
    {
        car_width = 0.8;
        car_length = 1.5;
        max_iter_time = 350;
        map_ = grid_map;

        grid_origin_x = grid_map.info.origin.position.x; //grid_map
        grid_origin_y = grid_map.info.origin.position.y;
        grid_reso = grid_map.info.resolution;
        grid_height = grid_map.info.height;
        grid_width = grid_map.info.width;
        max_index = grid_width * grid_height - 1;

        m_step = 1.0 / grid_reso;
        m_startPoint = new node();

        m_endPoint = new node();

        get_grid_coordinate_from_world(startPoint_.x, startPoint_.y, m_startPoint); //返回栅格地图的坐标
        get_grid_coordinate_from_world(endPoint_.x, endPoint_.y, m_endPoint);       //返回栅格地图的坐标
        m_startPoint->parent = NULL;
        m_endPoint->parent = NULL;

        nodeRandom = new node();
        m_nearestNode = new node();
        m_minRan = 1;
        m_maxRan = max_index - 1; //搜索范围是整个栅格地图
        m_nodeList.push_back(m_startPoint);
        //每次找的new_node与其对应的最近点的距离为1.0，即每次的新节点只能前进1.0
        search_radius = 2.0 / grid_reso; //每次得到一个新的newNode之后，需要在这个点周围search_radius以内找出所有点，这个距离越大，路径越平
                                         //然后计算newNode以这些点为父节点的路径代价，找出代价最小的
        expand_map(map_);
        vis_map_ = map_;
        cout << "RRT begins: " << endl;
        //cout<<"height "<<grid_height<<" width="<<grid_width<<" start x="<<m_startPoint->x
        //<<" y="<<m_endPoint->y<<endl;
    }

    // 有障礙物返回 true;
    bool RRT::check_collision_in_two_vector(const vector<RRT::node> &map_point_in_car, const nav_msgs::OccupancyGrid &grid_map)
    {
        car_width = 0.8;
        car_length = 1.5;
        max_iter_time = 350;
        map_ = grid_map;

        grid_origin_x = grid_map.info.origin.position.x; //grid_map
        grid_origin_y = grid_map.info.origin.position.y;
        grid_reso = grid_map.info.resolution;
        grid_height = grid_map.info.height;
        grid_width = grid_map.info.width;
        max_index = grid_width * grid_height - 1;

        m_step = 1.0 / grid_reso;
        m_startPoint = new node();

        m_endPoint = new node();

        

        nodeRandom = new node();
        m_nearestNode = new node();
        m_minRan = 1;
        m_maxRan = max_index - 1; //搜索范围是整个栅格地图
        m_nodeList.push_back(m_startPoint);
        //每次找的new_node与其对应的最近点的距离为1.0，即每次的新节点只能前进1.0
        search_radius = 2.0 / grid_reso; //每次得到一个新的newNode之后，需要在这个点周围search_radius以内找出所有点，这个距离越大，路径越平
                                         //然后计算newNode以这些点为父节点的路径代价，找出代价最小的
        expand_map(map_);
        vis_map_ = map_;
        cout << "RRT begins: " << endl;
        //cout<<"height "<<grid_height<<" width="<<grid_width<<" start x="<<m_startPoint->x
        //<<" y="<<m_endPoint->y<<endl;
        for(size_t i=0;i<map_point_in_car.size()-1;++i)
        {
            get_grid_coordinate_from_world(map_point_in_car[i].x, map_point_in_car[i].y, m_startPoint); //返回栅格地图的坐标
            get_grid_coordinate_from_world(map_point_in_car[i+1].x, map_point_in_car[i+1].y, m_endPoint);       //返回栅格地图的坐标
            m_startPoint->parent = NULL;
            m_endPoint->parent = NULL;
            int index_start=getIndexFromPoseInCarCoordinate(m_startPoint-> x, m_startPoint-> y);
            int index_end=getIndexFromPoseInCarCoordinate(m_endPoint-> x, m_endPoint-> y);
            if(index_start==-1||index_end==-1)  continue;
            bool hasObstacle=check_collision_between_two_grid_point(m_startPoint, m_endPoint);
            if(hasObstacle) return true;

        }
        return false;
    }

    void RRT::expand_map(nav_msgs::OccupancyGrid &map)
    {
        //cout<<"expanding map"<<endl;
        int radius = car_width / 2 / grid_reso;
        for (int xi = 0; xi < grid_width; ++xi)
        {
            for (int yi = 0; yi < grid_height; ++yi)
            {
                int index = xi + yi * grid_width;
                //cout<<"index ="<<index<<endl;
                if ((index < m_maxRan) && (index > 0) && map.data[index] == 100)
                {
                    //cout<<"expand"<<endl;
                    for (int m = xi - radius; m < xi + radius; ++m)
                    {
                        for (int n = yi - radius; n < yi + radius; ++n)
                        {
                            int index_new = m + n * grid_width;
                            if ((index_new < m_maxRan) && (index_new > 0))
                                map.data[index_new] = 90;
                        }
                    }
                    map.data[index] = 100;
                }
            }
        }
    }

    RRT::~RRT()
    {
        vector<node *>::iterator deletePtr = m_nodeList.begin();
        vector<node *>::iterator delePath = m_path.begin();
        for (; deletePtr != m_nodeList.end(); ++deletePtr)
        {
            if (*deletePtr != NULL)
            {
                delete *deletePtr;
                *deletePtr = NULL;
            }
        }

        for (; delePath != m_path.end(); ++delePath) //因为m_path里面的点一定在m_nodeList里面，所以不能再次析构m_path
        {
            *delePath = NULL;
        }
        m_nodeList.clear();
        m_path.clear();
        delete m_startPoint;  //保存路径的起点
        delete m_endPoint;    //保存路径的终点
        delete m_nearestNode; //保存最近点
        delete nodeRandom;    //定义一个随机点变量来保存随机点        cout << "RRT ends!!!!!!!" << endl;
    }

    void RRT::node_path_clear()
    {
        vector<node *>::iterator deletePtr = m_nodeList.begin();
        vector<node *>::iterator delePath = m_path.begin();
        for (; deletePtr != m_nodeList.end(); ++deletePtr)
        {
            if (*deletePtr != NULL)
            {
                delete *deletePtr;
                *deletePtr = NULL;
            }
        }

        for (; delePath != m_path.end(); ++delePath) //因为m_path里面的点一定在m_nodeList里面，所以不能再次析构m_path
        {
            *delePath = NULL;
        }
        m_nodeList.clear();
        m_path.clear();
    }

    int RRT::getIndexFromPoseInCarCoordinate(float x, float y)
    {
        node grid_coordinate;
        grid_coordinate.x = (x - grid_origin_x) / grid_reso;
        grid_coordinate.y = (y - grid_origin_y) / grid_reso;
        int index = grid_coordinate.y * grid_width + grid_coordinate.x;
        if((index <= m_minRan) ||(index >= m_maxRan))   return -1;//
        return index;
    }

    void RRT::get_grid_coordinate_from_world(float x, float y, RRT::node *grid_coordinate) //返回栅格地图的坐标
    {
        grid_coordinate->x = (x - grid_origin_x) / grid_reso;
        grid_coordinate->y = (y - grid_origin_y) / grid_reso;
    }
    void RRT::get_grid_coordinate_from_index(int grid_index, RRT::node *grid_coordinate) //由栅格地图index得到栅格地图坐标
    {
        grid_index = (grid_index <= m_minRan) ? m_minRan : grid_index;
        grid_index = (grid_index >= m_maxRan) ? m_maxRan : grid_index;
        grid_coordinate->y = grid_index % grid_width;
        grid_coordinate->x = grid_index - grid_coordinate->y * grid_width;
    }
    void RRT::get_world_coordinate(const RRT::node *grid_coordinate, RRT::node *world_coordinate) //由栅格地图坐标得到车辆坐标系下坐标
    {
        world_coordinate->x = grid_coordinate->x * grid_reso + grid_origin_x;
        world_coordinate->y = grid_coordinate->y * grid_reso + grid_origin_y;
    }

    int RRT::get_index_from_grid(const RRT::node &node1)
    {

        int index = node1.y * grid_width + node1.x;
        //cout<<"index ="<<index<<endl;
        index = (index <= m_minRan) ? m_minRan : index;
        index = (index >= m_maxRan) ? m_maxRan : index;
        //cout<<"max index="<<m_maxRan<<"  height*width="<<grid_height*grid_width<<endl;
        return index;
    }
    void RRT::GetRanNode(RRT::node *randNode)
    {
        static int i = 0;
        srand((unsigned)time(NULL) * i);
        i++;
        //rand() / double(RAND_MAX);
        int index = rand() / double(RAND_MAX) * (m_maxRan - m_minRan) + m_minRan;
        get_grid_coordinate_from_index(index, randNode); //由栅格地图index得到栅格地图坐标
    }

    vector<RRT::node> RRT::bresenham_k_lessthan_1(int x0, int y0, int x1, int y1)
    {

        int dx = x1 - x0;
        int dy = y1 - y0;

        vector<node> node_vec;
        node new_node = node(x0, y0);
        node_vec.push_back(new_node);

        int p = 2 * dy - dx;
        int xi = x0;
        int yi = y0;

        for (xi = x0; xi <= x1; ++xi)
        {
            node_vec.push_back(node(xi, yi));
            //计算下一个p
            if (p > 0)
            {
                p = p + 2 * (dy - dx);
                yi = yi + 1;
            }
            else
                p = p + 2 * dy;
        }
        return node_vec;
    }

    //所有象限测试，完全没有问题
    vector<RRT::node> RRT::bresenham_check(int x0, int y0, int x1, int y1)
    {

        //将直线朝向为二三象限的点全部转换到一四象限
        if (x0 > x1)
        {
            int tmp = x0;
            x0 = x1;
            x1 = tmp;
            tmp = y1;
            y1 = y0;
            y0 = tmp;
        }

        int dy = y1 - y0;
        int dx = x1 - x0;

        vector<node> node_vec;
        if (dy > 0) //表示一象限
        {
            if (abs(dx) > abs(dy)) //第一象限斜率小于1部分
            {
                node_vec = bresenham_k_lessthan_1(x0, y0, x1, y1);
                return node_vec;
            }

            else //第一象限斜率大于1部分
            {

                node_vec = bresenham_k_lessthan_1(y0, x0, y1, x1);
                for (size_t ii = 0; ii < node_vec.size(); ++ii)
                {
                    float tmp_x = node_vec[ii].x;
                    node_vec[ii].x = node_vec[ii].y;
                    node_vec[ii].y = tmp_x;
                }
                return node_vec;
            }
        }
        else //四象限
        {
            if (abs(dx) > abs(dy))
            {
                node_vec = bresenham_k_lessthan_1(x0, -1 * y0, x1, -1 * y1);
                for (size_t ii = 0; ii < node_vec.size(); ++ii)
                {
                    node_vec[ii].y = -1 * node_vec[ii].y;
                }
                return node_vec;
            }

            else
            {
                node_vec = bresenham_k_lessthan_1(-1 * y0, x0, -1 * y1, x1);
                for (size_t ii = 0; ii < node_vec.size(); ++ii)
                {
                    node_vec[ii].x = -1 * node_vec[ii].x;
                }

                for (size_t ii = 0; ii < node_vec.size(); ++ii)
                {
                    float tmp_x = node_vec[ii].x;
                    node_vec[ii].x = node_vec[ii].y;
                    node_vec[ii].y = tmp_x;
                }
                return node_vec;
            }
        }
    }

    //有碰撞，则返回true,经过测试
    bool RRT::check_collision_between_two_grid_point(const RRT::node *node1, const RRT::node *node2)
    {
        // cout<<"start bresenham.."<<endl;
        vector<node> node_vec = bresenham_check(node1->x, node1->y, node2->x, node2->y);

        //将path_的值附为node_vec，然后节点里面进行可视化调试bresenham
        //for (const auto &node_a : node_vec)
        //{
        //int index_now=get_index_from_grid(node_a);
        //vis_map_.data[index_now]=60;

        //}
        //cout<<"end bresenham"<<endl;
        for (auto node_a : node_vec)
        {
            int index = get_index_from_grid(node_a);
            //cout<<" after index= "<<index<<endl;
            if ((index < m_maxRan) && map_.data[index] > 60)
                return true;
        }
        return false;
    }

    //给生成的随机节找在已经生成的随机数中找出一个最近点
    void RRT::GetNearestNode(const RRT::node *randNode)
    {
        float distTmp;
        float dxTmp;
        float dyTmp;
        float distMax = 9999.9;

        for (auto p : m_nodeList) // list只能通过迭代器进行遍历，vector还可以数组方式遍历。
        {
            dxTmp = p->x - randNode->x;
            dyTmp = p->y - randNode->y;
            distTmp = sqrt(dxTmp * dxTmp + dyTmp * dyTmp);
            if ((distTmp < distMax) && !(check_collision_between_two_grid_point(p, randNode)))
            {
                distMax = distTmp;
                m_nearestNode = p;
            }
        }
    }

    //!!! 这里选点的时候应该对两个点的连线进行碰撞检测
    void RRT::find_neighbor_node_for_newNode(RRT::node *newNode, vector<int> &nearNodeIndexIn_m_nodeList)
    {
        float distTmp;
        float dxTmp;
        float dyTmp;
        for (size_t i = 0; i < m_nodeList.size(); ++i)
        //for (auto p : m_nodeList) // list只能通过迭代器进行遍历，vector还可以数组方式遍历。
        {
            node *p = m_nodeList[i];
            dxTmp = p->x - newNode->x;
            dyTmp = p->y - newNode->y;
            distTmp = sqrt(dxTmp * dxTmp + dyTmp * dyTmp);
            if (distTmp < search_radius && !(check_collision_between_two_grid_point(p, newNode)))
            {

                nearNodeIndexIn_m_nodeList.push_back(i);
            }
        }
    }
    //在半径为search_radius的圆内找到一个最好的父节点
    void RRT::search_best_parent(RRT::node *newNode, const vector<int> &nearNodeIndexIn_m_nodeList)
    {
        float distTmp;
        float dxTmp;
        float dyTmp;
        for (size_t i = 0; i < nearNodeIndexIn_m_nodeList.size(); ++i)
        //for (auto p : m_nodeList) // list只能通过迭代器进行遍历，vector还可以数组方式遍历。
        {
            node *p = m_nodeList[nearNodeIndexIn_m_nodeList[i]];
            dxTmp = p->x - newNode->x;
            dyTmp = p->y - newNode->y;
            distTmp = sqrt(dxTmp * dxTmp + dyTmp * dyTmp);

            float new_cost = p->cost + distTmp;
            if (newNode->cost > new_cost) //新结点的代价之更大
            {
                newNode->cost = new_cost;
                newNode->parent = p;
            }
        }
    }

    //查看newNode周围的点是否能用newNode作为父节点
    void RRT::reWire_For_NewNode_Neighbor(RRT::node *newNode, vector<int> &nearNodeIndexIn_m_nodeList)
    {
        float distTmp;
        float dxTmp;
        float dyTmp;
        for (size_t i = 0; i < nearNodeIndexIn_m_nodeList.size(); ++i)
        //for (auto p : m_nodeList) // list只能通过迭代器进行遍历，vector还可以数组方式遍历。
        {
            node *p = m_nodeList[nearNodeIndexIn_m_nodeList[i]];
            dxTmp = p->x - newNode->x;
            dyTmp = p->y - newNode->y;
            distTmp = sqrt(dxTmp * dxTmp + dyTmp * dyTmp);
            float new_parent_cost = p->cost + distTmp;
            if (p->cost > new_parent_cost)
            {
                p->cost = new_parent_cost;
                p->parent = newNode;
            }
        }
    }

    nav_msgs::Path RRT::GetPath()
    {

        float prob;
        float theta;

        float dx;
        float dy;
        float dist;
        int count = 0;
        int iter_time = max_iter_time;
        bool find_path = false;
        //vector<node *> nearNodeList;
        while (--iter_time)
        {
            ++count;
            //cout<<"***"<<endl;
            static int i = 0;
            srand((unsigned)time(NULL) * i);
            i++;
            prob = rand() / double(RAND_MAX);
            //cout<<"prob="<<prob<<endl;

            if (prob > 0.2) //prob的概率能够直接选到终点
            {
                GetRanNode(nodeRandom);
            }
            else
            {
                nodeRandom->x = m_endPoint->x;
                nodeRandom->y = m_endPoint->y;
                nodeRandom->cost = m_endPoint->cost;
            }

            GetNearestNode(nodeRandom);

            theta = atan2f(nodeRandom->y - m_nearestNode->y, nodeRandom->x - m_nearestNode->x);
            //cout<<theta<<endl;
            //cout<<m_nearestNode->x<<"***"<<m_nearestNode->y<<endl;
            node *newNode = new node();
            newNode->x = m_nearestNode->x + m_step * cosf(theta); //在距离最近的点与rand node间生成一个最近点
            newNode->y = m_nearestNode->y + m_step * sinf(theta);
            // cout<<"newNode: "<<"( "<<newNode->x<<","<<newNode->y<<" )"<<endl;
            newNode->parent = m_nearestNode;                //将newNode的父节点赋为这个最近点
            newNode->cost = newNode->parent->cost + m_step; //每次只走m_step

            if (check_collision_between_two_grid_point(newNode, newNode->parent)) //如果有碰撞，则新生成的这个节点无效，重新搜索
            {
                delete newNode;
                continue;
            }

            m_nodeList.push_back(newNode); //无碰撞，则加入到list里面

            /*****************RRT星操作*********************/
            //如果是RRT*，则需要在半径为R的圆内找到多个点，然后依次算所有的路径的cost，选一个cost最小的  这里会溢出

            vector<int> nearNodeIndexIn_m_nodeList; //保存距离newNode较近的节点
            find_neighbor_node_for_newNode(newNode, nearNodeIndexIn_m_nodeList);
            search_best_parent(newNode, nearNodeIndexIn_m_nodeList); //在半径为search_radius的圆内找到一个最好的父节点
            reWire_For_NewNode_Neighbor(newNode, nearNodeIndexIn_m_nodeList);
            while (nearNodeIndexIn_m_nodeList.size() > 0)
            {
                nearNodeIndexIn_m_nodeList.pop_back();
            }
            /***********************************************/

            //检验其余终点的间距，如果小于1.0米，则表示找到终点
            dx = newNode->x - m_endPoint->x;
            dy = newNode->y - m_endPoint->y;
            dist = sqrt(dx * dx + dy * dy);

            if ((dist < m_step) && !check_collision_between_two_grid_point(newNode, m_endPoint)) //与中点距离小于1米，则表示已经找到最优路径。这里需要加一个碰撞检测
            {

                m_endPoint->parent = newNode;

                //为终点找一个更好的父节点
                vector<int> nearNodeIndexIn_m_nodeList; //保存距离newNode较近的节点
                find_neighbor_node_for_newNode(m_endPoint, nearNodeIndexIn_m_nodeList);
                search_best_parent(m_endPoint, nearNodeIndexIn_m_nodeList); //在半径为search_radius的圆内找到一个最好的父节点

                m_nodeList.push_back(m_endPoint);
                cout << "FIND PATH!!!!!!!!!!!!!!!!!" << endl;
                find_path = true;
                break;
            }
            //cout<<count<<" ";
        }
        cout << "random search count= " << count << endl;
        nav_msgs::Path path_;
        if (!find_path)
        {
            cout << " NOT FIND PATH!!!!" << endl;
            node_path_clear();
            return path_;
        }
        node *last;
        last = m_endPoint;
        node *world_coordinate = new node();
        //cout << "1" << endl;
        /*while(path_.pose.size()>0)
        {
            path_.pose.pop_back();
        }*/

        int path_size = 0;
        while (last != NULL)
        {
            m_path.push_back(last); //是逆着打印的

            geometry_msgs::PoseStamped this_pose_stamped;

            //将栅格地图坐标转为世界坐标系
            get_world_coordinate(last, world_coordinate); //由栅格地图坐标得到车辆坐标系下坐标

            this_pose_stamped.pose.position.x = world_coordinate->x;
            this_pose_stamped.pose.position.y = world_coordinate->y;
            this_pose_stamped.pose.position.z = 0;
            path_.poses.push_back(this_pose_stamped);
            last = last->parent;
            ++path_size;
        }
        delete world_coordinate;

        // m_path.push_back()
        //m_path.push_back(m_startPoint);
        //cout << "print the path!!!!   " << m_path.size() << " path size=" << path_size << " node_list size=" << m_nodeList.size() << endl;

        for (auto p : m_path)
        {
            //cout << "( " << int(p->x) << "," << int(p->y) << " )" << endl;
            int index_now = get_index_from_grid(*p);
            //vis_map_.data[index_now] = 60;
        }

        node_path_clear();

        return path_;
    }

} // namespace lidar_localization
