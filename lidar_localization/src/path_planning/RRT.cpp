#include "lidar_localization/path_planning/RRT.hpp"
#include <unistd.h>

namespace lidar_localization
{

    RRT::~RRT()
    {
        /*
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
        delete m_nearestNode; //保存最近点*/
    }

    void RRT::node_path_clear()
    {
       /* vector<node *>::iterator deletePtr = m_nodeList.begin();
        for (; deletePtr != m_nodeList.end(); ++deletePtr)
        {
            if (*deletePtr != NULL)
            {
                delete *deletePtr;
                *deletePtr = NULL;
            }
        }
        */
     
    }

    void RRT::get_grid_coordinate_from_car(float x, float y, RRT::node* grid_coordinate) //返回栅格地图的坐标
    {
        grid_coordinate->x = int((x - grid_origin_x) / grid_reso);
        grid_coordinate->y = int((y - grid_origin_y) / grid_reso);
    }
    void RRT::get_grid_coordinate_from_index(int grid_index, RRT::node *grid_coordinate) //由栅格地图index得到栅格地图坐标
    {
        grid_index = (grid_index <= m_minRan) ? m_minRan : grid_index;
        grid_index = (grid_index >= m_maxRan) ? m_maxRan : grid_index;
        grid_coordinate->y = grid_index % grid_width;
        grid_coordinate->x = grid_index - grid_coordinate->y * grid_width;
    }
    void RRT::get_car_coordinate(const RRT::node *grid_coordinate, RRT::node *car_coordinate) //由栅格地图坐标得到车辆坐标系下坐标
    {
        car_coordinate->x = grid_coordinate->x * grid_reso + grid_origin_x;
        car_coordinate->y = grid_coordinate->y * grid_reso + grid_origin_y;
    }
    //计算index不应该进行边界的判断
    int RRT::get_index_from_grid(const RRT::node &node1)
    {

        int index = node1.y * grid_width + node1.x;
        return index;
    }
    
 int RRT::getIndexFromPoseInCarCoordinate(const float x,const float y)
    {
        node grid_coordinate;
        grid_coordinate.x = (x - grid_origin_x) / grid_reso;
        grid_coordinate.y = (y - grid_origin_y) / grid_reso;
        int index = grid_coordinate.y * grid_width + grid_coordinate.x;
        if((index <= m_minRan) ||(index >= m_maxRan))   return -1;
        return index;
    }    

    void RRT::GetRanNode(RRT::node *randNode)
    {
        static int i = 0;
        srand((unsigned)time(NULL) * i);
        i++;
        //rand() / double(RAND_MAX);
        randNode->x= rand() / double(RAND_MAX) * (rand_x_max - rand_x_min) + rand_x_min;
        srand((unsigned)time(NULL) * i);
        i++;
        randNode->y = rand() / double(RAND_MAX) * (rand_y_max - rand_y_min) + rand_y_min;
        //cout<<"random node x="<<randNode->x<<" y="<<randNode->y<<endl;
    }


 // 有障礙物返回 true;输入的点是车辆坐标系下的点
    bool RRT::check_collision_in_two_vector(const vector<RRT::node> &map_point_in_car, const nav_msgs::OccupancyGrid &grid_map)
    {


        grid_map_obstacle_value=WALKING_SPACE;//全局路径点的障碍物的判断用80来判断，判断条件宽松一点。
             
        map_ = grid_map;

        grid_origin_x = grid_map.info.origin.position.x; //grid_map
        grid_origin_y = grid_map.info.origin.position.y;
        grid_reso = grid_map.info.resolution;
        grid_height = grid_map.info.height;
        grid_width = grid_map.info.width;
        max_index = grid_width * grid_height - 1;

        node* node1=new node();
        node* node2=new node();
      

        m_minRan = 1;
        m_maxRan = max_index - 1; //搜索范围是整个栅格地图
        //每次找的new_node与其对应的最近点的距离为1.0，即每次的新节点只能前进1.0

        bool hasObstacle=false;
        int map_point_size=map_point_in_car.size()-1;

        for(int i=0;i<map_point_size;++i)
        {
                      
            int index_start=getIndexFromPoseInCarCoordinate(map_point_in_car[i].x,map_point_in_car[i]. y);
            int index_end=getIndexFromPoseInCarCoordinate(map_point_in_car[i+1].x, map_point_in_car[i+1].y);
            if(index_start==-1||index_end==-1)
            {   
                ROS_WARN(" path point is out of grid map");
                continue;
            }  
            get_grid_coordinate_from_car(map_point_in_car[i].x, map_point_in_car[i].y, node1); //返回栅格地图的坐标
            get_grid_coordinate_from_car(map_point_in_car[i+1].x, map_point_in_car[i+1].y, node2);       //返回栅格地图的坐标
     

            hasObstacle=check_collision_between_two_grid_point(node1, node2);
            if(hasObstacle)
            {
                ROS_ERROR(" obstacle detected");
            }
            else
            {
                ROS_WARN(" obstacle not detected");
            }
            if(hasObstacle) break;

        }
        delete node1;
        delete node2;
        return hasObstacle;
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

    //有碰撞，则返回true,经过测试，
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
            if ((index < m_maxRan)&&(index>=0) && map_.data[index] >= grid_map_obstacle_value)
                return true;
        }
        return false;
    }


    //有碰撞，则返回true,经过测试，
    bool RRT::check_collision_between_two_car_point(const RRT::node *node1, const RRT::node *node2)
    {
        //车辆坐标系点转栅格坐标系点
        RRT::node* grid_coordinate1=new node();
        RRT::node* grid_coordinate2=new node();

        get_grid_coordinate_from_car(node1-> x, node1-> y,  grid_coordinate1); //返回栅格地图的坐标
        get_grid_coordinate_from_car(node2-> x, node2-> y,  grid_coordinate2); //返回栅格地图的坐标


        vector<node> node_vec = bresenham_check(grid_coordinate1->x, grid_coordinate1->y, grid_coordinate2->x, grid_coordinate2->y);

        //将path_的值附为node_vec，然后节点里面进行可视化调试bresenham
        //for (const auto &node_a : node_vec)
        //{
        //int index_now=get_index_from_grid(node_a);
        //vis_map_.data[index_now]=60;

        //}
        //cout<<"end bresenham"<<endl;
        bool collision_flag=false;
        for (auto node_a : node_vec)
        {
            int index = get_index_from_grid(node_a);
            if ((index < m_maxRan)&&(index>=0) && map_.data[index] >= grid_map_obstacle_value)
            {
                collision_flag=true;
                break;
            }
        }
        delete grid_coordinate1;
        delete grid_coordinate2;

        return collision_flag;
    }



    //给生成的随机节找在已经生成的随机数中找出一个最近点
    void RRT::GetNearestNode(const RRT::node *randNode,const vector<RRT::node*>&  m_nodeList,node *&m_nearestNode,bool& hasFindNearestPoint)
    {
        float distTmp;
        float dxTmp;
        float dyTmp;
        float distMax = 9999.9;
        hasFindNearestPoint=false;
       
        for (unsigned int i=0;i<m_nodeList.size();++i) // list只能通过迭代器进行遍历，vector还可以数组方式遍历。
        {
            node* p=m_nodeList[i];
            dxTmp = p->x - randNode->x;
            dyTmp = p->y - randNode->y;
            distTmp = sqrt(dxTmp * dxTmp + dyTmp * dyTmp);
            if ((distTmp < distMax) && (!(check_collision_between_two_car_point(p, randNode))))
            {
                hasFindNearestPoint=true;
                distMax = distTmp;
                m_nearestNode = p;
          
            }
        }
    }

    //!!! 这里选点的时候应该对两个点的连线进行碰撞检测
    void RRT::find_neighbor_node_for_newNode(RRT::node *newNode, vector<int> &nearNodeIndexIn_m_nodeList,const vector<RRT::node*>&  m_nodeList)
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
            if (distTmp < search_radius && !(check_collision_between_two_car_point(p, newNode)))
            {

                nearNodeIndexIn_m_nodeList.push_back(i);
            }
        }
    }
    //在半径为search_radius的圆内找到一个最好的父节点
    void RRT::search_best_parent(RRT::node *newNode, const vector<int> &nearNodeIndexIn_m_nodeList,const vector<RRT::node*>&  m_nodeList)
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
    void RRT::reWire_For_NewNode_Neighbor(RRT::node *newNode, vector<int> &nearNodeIndexIn_m_nodeList, vector<RRT::node*>&  m_nodeList)
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


    
    //输入坐标是车辆坐标系下坐标，里面RRT操作是在栅格坐标系下进行
    void RRT::init( const nav_msgs::OccupancyGrid &grid_map)
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
        max_index = grid_width * grid_height ;

        m_step = 0.5;


        m_minRan = 1;
        m_maxRan = max_index - 1; //搜索范围是整个栅格地图
        

        rand_x_min=0+0.3;//随即点的搜索范围
        rand_x_max=grid_width*grid_reso-0.3;
        rand_y_min=-grid_height*grid_reso/2+0.3;
        rand_y_max=grid_height*grid_reso/2-0.3;
        //cout<<" rand xmin ="<<rand_x_min<<" xmax ="<<rand_x_max<<" ymin ="<<rand_y_min<<" ymax ="<<rand_y_max<<endl;

        //每次找的new_node与其对应的最近点的距离为1.0，即每次的新节点只能前进1.0
        search_radius = 2.0 ; //每次得到一个新的newNode之后，需要在这个点周围search_radius以内找出所有点，这个距离越大，路径越平
                                         //然后计算newNode以这些点为父节点的路径代价，找出代价最小的
        vis_map_ = map_;

    }

    //表示规划时需要避开的栅格地图的障碍物值。这个值在规划的时候首先取PLANNING_SPACE=70进行规划，尽可能的离障碍物远一点
                //如果没有路径，则用WALKING_SPACE=80进行规划。但是在用two_vector进行全局路径判断障碍物时仍用WALKING_SPACE=80来进行判断
    nav_msgs::Path RRT::GetPath(const RRT::node &startPoint_, const RRT::node &endPoint_)
    {
        grid_map_obstacle_value=PLANNING_SPACE;
        nav_msgs::Path  planning_path=GetPlanningPath(startPoint_, endPoint_);
        if(planning_path.poses.size()==0)  
        {
            grid_map_obstacle_value=WALKING_SPACE;
            planning_path=GetPlanningPath(startPoint_, endPoint_);        
        }
        return planning_path;
    }
    

    //输出一个基于车辆坐标系下的目标点,grid_map_obstacle_value表示需要定义的规划的障碍物在栅格地图中的值，>=grid_map_obstacle_value的栅格需要避开
    nav_msgs::Path RRT::GetPlanningPath(const RRT::node &startPoint_, const RRT::node &endPoint_)
    {
        my_mutext.lock();
        node *m_startPoint = new node();//保存路径的起点
        
        m_startPoint->x=startPoint_.x;
        m_startPoint->y=startPoint_.y;
        m_startPoint->cost=0.0;
        m_startPoint->parent = NULL;
    
        node *m_endPoint = new node();//保存路径的终点
        m_endPoint->x=endPoint_.x;
        m_endPoint->y=endPoint_.y;
        m_endPoint->parent = m_startPoint;

        //!!!m_nearestNode不能new，因为这个指针变量只能指向一个已有的node变量。如果new了之后，就会初始化一个新的变量，如果第一次没有为nodeRandom在m_nodeList
        //里面找到一个与其连线无障碍物的点，那么m_nearestNode就还是初始值，并且parent为NULL。然后就会进入下面计算newNode的步骤，然后起点就变成newNode了
        //这样写还有一个缺陷，找不到最近点的时候，如果继续进行下面的操作，则m_nearestNode指向为空，继续下面的操作就会核心转储

        //node *m_nearestNode = new node();//保存找到的最近点  
        node *m_nearestNode;//保存找到的最近点

        cout << "RRT begins: " << endl;
        //cout<<" car cordinate start_p x="<<m_startPoint->x<<" y="<<m_startPoint->y<<" end_p x="<<m_endPoint->x<<" y="<<m_endPoint->y<<endl;

        vector<node *> m_nodeList; //保存已经搜寻到的点,改成局部变量

        
        float prob;
        float theta;

        float dx;
        float dy;
        float dist;
        int count = 0;
        int iter_time = max_iter_time;
        bool find_path = false;
        //vector<node *> nearNodeList;
        //cout<<" RRT before search:"<<endl;
        //cout<<" grid cordinate start_p x="<<m_startPoint->x<<" y="<<m_startPoint->y<<" end_p x="<<m_endPoint->x<<" y="<<m_endPoint->y;
        //cout<<"  m_nodeList size= "<<m_nodeList.size()<<endl;
        m_nodeList.push_back(m_startPoint);
        while (--iter_time)
        {
            node *nodeRandom=new node();    //定义一个随机点变量来保存随机点
            ++count;
            //cout<<"***"<<endl;
            static int i = 0;
            srand((unsigned)time(NULL) * i);
            i++;
            prob = rand() / double(RAND_MAX);
            //cout<<"prob="<<prob<<endl;

            if (prob > 0.2)//prob的概率能够直接选到终点
            {
                GetRanNode(nodeRandom);
            }
            else
            {
                nodeRandom->x = m_endPoint->x;
                nodeRandom->y = m_endPoint->y;
                nodeRandom->cost = m_endPoint->cost;
                nodeRandom->parent = m_endPoint->parent;
            }
            bool hasFindNearestPoint=false;
            GetNearestNode(nodeRandom,m_nodeList,m_nearestNode,hasFindNearestPoint);
            if(!hasFindNearestPoint) continue;



            //cout<<"find nearest point"<<endl;



            theta = atan2f(nodeRandom->y - m_nearestNode->y, nodeRandom->x - m_nearestNode->x);
            delete nodeRandom;
            //cout<<theta<<endl;
            //cout<<m_nearestNode->x<<"***"<<m_nearestNode->y<<endl;
            node *newNode = new node();
            newNode->x = m_nearestNode->x + m_step * cosf(theta); //在距离最近的点与rand node间生成一个最近点
            newNode->y = m_nearestNode->y + m_step * sinf(theta);
            //cout<<"newNode: "<<"( "<<newNode->x<<","<<newNode->y<<" )"<<endl;
            newNode->parent = m_nearestNode;                //将newNode的父节点赋为这个最近点
            newNode->cost = m_nearestNode->cost + m_step; //每次只走m_step

            if (check_collision_between_two_car_point(newNode, newNode->parent)) //如果有碰撞，则新生成的这个节点无效，重新搜索
            {
                delete newNode;
                continue;
            }



            //cout<<" push back in m_nodeList: newNode x="<<newNode->x<<" y="<<newNode->y<<"  parent m_nearestNode x="<<m_nearestNode->x<<"  y="<<m_nearestNode->y<<endl;
            m_nodeList.push_back(newNode); //无碰撞，则加入到list里面


            /*****************RRT星操作*********************/
            //如果是RRT*，则需要在半径为R的圆内找到多个点，然后依次算所有的路径的cost，选一个cost最小的  这里会溢出

            vector<int> nearNodeIndexIn_m_nodeList; //保存距离newNode较近的节点
            find_neighbor_node_for_newNode(newNode, nearNodeIndexIn_m_nodeList,m_nodeList);
            search_best_parent(newNode, nearNodeIndexIn_m_nodeList,m_nodeList); //在半径为search_radius的圆内找到一个最好的父节点
            reWire_For_NewNode_Neighbor(newNode, nearNodeIndexIn_m_nodeList,m_nodeList);
            while (nearNodeIndexIn_m_nodeList.size() > 0)
            {
                nearNodeIndexIn_m_nodeList.pop_back();
            }
            /***********************************************/

            //检验其余终点的间距，如果小于1.0米，则表示找到终点
            dx = newNode->x - m_endPoint->x;
            dy = newNode->y - m_endPoint->y;
            dist = sqrt(dx * dx + dy * dy);

            if ((dist < m_step) && !check_collision_between_two_car_point(newNode, m_endPoint)) //与中点距离小于1米，则表示已经找到最优路径。这里需要加一个碰撞检测
            {

                m_endPoint->parent = newNode;

                
                //为终点找一个更好的父节点
                vector<int> nearNodeIndexIn_m_nodeList; //保存距离newNode较近的节点
                find_neighbor_node_for_newNode(m_endPoint, nearNodeIndexIn_m_nodeList,m_nodeList);
                search_best_parent(m_endPoint, nearNodeIndexIn_m_nodeList,m_nodeList); //在半径为search_radius的圆内找到一个最好的父节点
                
                m_nodeList.push_back(m_endPoint);
                cout << "FIND PATH!!!!!!!!!!!!!!!!!" << endl;
                find_path = true;
                break;
            }
            //cout<<count<<" ";
        }
        cout << "random search count= " << count << endl;

        my_mutext.unlock();
        nav_msgs::Path path_;
        if (!find_path)
        {
            cout << " NOT FIND PATH!!!!" << endl;

            vector<node *>::iterator deletePtr = m_nodeList.begin();
            for (; deletePtr != m_nodeList.end(); ++deletePtr)
            {
                if (*deletePtr != NULL)
                {
                    delete *deletePtr;
                    *deletePtr = NULL;
                }
            }


            node_path_clear();
            return path_;
        }
        node *last;
        last = m_endPoint;
        //cout << "1" << endl;
        /*while(path_.pose.size()>0)
        {
            path_.pose.pop_back();
        }*/
        //cout<<" m_nodeList  first x= "<<m_nodeList[i]->x<<" y="<<m_nodeList[i]->y<<endl;

        int path_size = 0;
        while (last !=NULL )
        {

            geometry_msgs::PoseStamped this_pose_stamped;
            this_pose_stamped.pose.position.x = last->x;
            this_pose_stamped.pose.position.y = last->y;
            this_pose_stamped.pose.position.z = 0;
            path_.poses.push_back(this_pose_stamped);

            last = last->parent;
            ++path_size;
        }
        reverse(path_.poses.begin(),path_.poses.end());
        //for(unsigned int i=0;i<path_.poses.size();++i)
        //    cout<<" car map coordinate x="<<path_.poses[i].pose.position.x<<" y="<<path_.poses[i].pose.position.y<<endl;

       /* cout<<"m_nodeList :"<<endl;
        for(unsigned int i=0;i<m_nodeList.size();++i)
        {   
            cout<<" x= "<<m_nodeList[i]->x<<" y= "<<m_nodeList[i]->y;
            if(m_nodeList[i]->parent==NULL)
            {
                cout<<"; parent is NULL";
                
            }
            else if(m_nodeList[i]->parent->parent==NULL)
            {

                cout<<" ; grandparent is NULL :parent x="<<m_nodeList[i]->parent->x<<" y= "<<m_nodeList[i]->parent->y;
            }
            cout<<endl;
        }*/
            



        //cout<<"RRT end:"<<endl;
        //cout<<" car cordinate start_p x="<<startPoint_.x<<" y="<<startPoint_.y<<" end_p x="<<endPoint_.x<<" y="<<endPoint_.y<<endl;
        //cout<<" grid cordinate start_p x="<<m_startPoint->x<<" y="<<m_startPoint->y<<" end_p x="<<m_endPoint->x<<" y="<<m_endPoint->y;
        //if(m_startPoint->parent==NULL)  //cout<<"  start parent is NULL";
        //cout<<endl;
        vector<node *>::iterator deletePtr = m_nodeList.begin();
        for (; deletePtr != m_nodeList.end(); ++deletePtr)
        {
            if (*deletePtr != NULL)
            {
                delete *deletePtr;
                *deletePtr = NULL;
            }
        }

        node_path_clear();
        //cout<<endl<<endl<<endl<<endl;
        return path_;
    }

} // namespace lidar_localization
