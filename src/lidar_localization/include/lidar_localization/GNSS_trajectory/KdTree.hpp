#ifndef KD_TREE_HPP_
#define KD_TREE_HPP_

#include "ros/ros.h"
#include <cmath>
#include <iostream>
#include <string>
#include <tf2_ros/transform_listener.h>
#include <vector>
#include <mutex>

#include "Geocentric/LocalCartesian.hpp"
#include "lidar_localization/sensor_data/gnss_data.hpp"

#include <iostream>
#include <vector>
#include <algorithm>
#include <string>
#include <cmath>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
using namespace std;
namespace lidar_localization
{

    class myKdTree
    {

    public:
        struct Position
        {

            float x;
            float y;
            float heading;
            Position(float _x = 0.0f, float _y = 0.0f, float _heading = 0.0f) : x(_x), y(_y), heading(_heading) {}
            Position &operator=(const Position &position)
            {
                x = position.x;
                y = position.y;
                heading = position.heading;
                return *this;
            }
        };

        class KdTree
        {
        public:
            vector<Position> root;
            KdTree *parent;
            KdTree *leftChild;
            KdTree *rightChild;
            //默认构造函数
            KdTree() { parent = leftChild = rightChild = NULL; }
            bool isSame(const vector<Position> &p1, const vector<Position> &p2)
            {
                if (p1.size() != p2.size())
                    return false;

                int size = p1.size();

                for (int i = 0; i < size; i++)
                {
                    if (abs(p2[i].x - p1[i].x) > 0.0001 || abs(p2[i].y - p1[i].y) > 0.0001 || abs(p2[i].heading - p1[i].heading) > 0.0001)
                        return false;
                }
                return true;
            }
            //判断kd树是否为空
            bool isEmpty()
            {
                return root.empty(); // 如果是空，则返回true.
            }
            //判断kd树是否只是一个叶子结点
            bool isLeaf()
            {
                return (!root.empty()) &&
                       rightChild == NULL && leftChild == NULL;
            }
            //判断是否是树的根结点
            bool isRoot()
            {
                return (!isEmpty()) && parent == NULL;
            }
            //判断该子kd树的根结点是否是其父kd树的左结点
            bool isLeft()
            {
                return isSame(parent->leftChild->root, root); // vector判断相等，就是里面的数据相等。
            }
            //判断该子kd树的根结点是否是其父kd树的右结点
            bool isRight()
            {
                return isSame(parent->rightChild->root, root); //通过结点的数值来判断，会不会出现数值相同的结点？
            }
        };

    private:
        vector<Position> data;
        KdTree *tree;

    private:
        float measureDistance(const Position &goal, const Position &point);
        float FindMiddleValue(const vector<Position> &_data, const unsigned depth);
        void BuildKdTree(KdTree *_tree, const vector<Position> data, const unsigned depth);

        void DestroyTree(KdTree *_tree);

    public:
        myKdTree(const vector<Position> &_data);
        void GetKdTree(const unsigned depth);
        Position searchNearestPoint(const Position &goal);
        ~myKdTree();
    };

} // namespace lidar_localization
#endif