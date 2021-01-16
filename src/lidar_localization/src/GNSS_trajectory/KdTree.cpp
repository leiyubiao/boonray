#include "lidar_localization/GNSS_trajectory/KdTree.hpp"
namespace lidar_localization
{

    myKdTree::myKdTree(const vector<myKdTree::Position> &_data)
    {

        if (_data.size() == 0)
        {
            cout << "warning: no data!!!" << endl;
        }

        tree = new KdTree();
        for (auto p : _data)
            data.push_back(p);
        cout << "constructed!!!" << endl;
    }

    void myKdTree::DestroyTree(KdTree *_tree)
    {
        if (_tree == NULL)
            return;
        DestroyTree(_tree->leftChild);
        DestroyTree(_tree->rightChild);

        delete _tree;
        _tree = NULL;
    }

    float myKdTree::measureDistance(const myKdTree::Position &goal, const myKdTree::Position &point)
    {
        float distance;
        float dx = goal.x - point.x;
        float dy = goal.y - point.y;
        distance = sqrt(dx * dx + dy * dy);
        return distance;
    }

    float myKdTree::FindMiddleValue(const vector<myKdTree::Position> &_data, const unsigned depth)
    {
        int size = _data.size();
        vector<float> tmp;
        if (depth == 0)
        {
            for (int i = 0; i < size; i++)
            {
                tmp.push_back(data[i].x);
            }
        }
        else if (depth == 1)
        {
            for (int i = 0; i < size; i++)
            {
                tmp.push_back(data[i].y);
            }
        }

        sort(tmp.begin(), tmp.end());
        //cout<<"middle: "<<tmp[size/2]<<endl;
        /*for (auto p:tmp)
        cout<<p<<endl;*/
        return tmp[size / 2];
    }

    void myKdTree::BuildKdTree(KdTree *tree, const vector<myKdTree::Position> data, const unsigned depth)
    {
        unsigned sampleNum = data.size();
        //cout<<"sampleNum: "<<sampleNum<<endl;

        // 递归的终止条件
        if (sampleNum == 0)
            return;
        if (sampleNum == 1)
        {
            tree->root.push_back(data[0]);
            return; // 假设只有一个数据时。
        }
        
        cout<<"sampleNum: "<<sampleNum<<endl;
        unsigned k = 2;
        unsigned splitAttribute = depth % k; // 0是x, 1是y。
        float splitMidValue = FindMiddleValue(data, splitAttribute);
       // cout<<" splitAttribute: "<<splitMidValue<<endl;

        // 根据选定的切分属性与切分值，将数据分为两个子集。
        vector<Position> subSet1;
        vector<Position> subSet2;
        for (unsigned i = 0; i < sampleNum; ++i)
        {
            if (splitAttribute == 0)
            {
                if (abs(splitMidValue - data[i].x) < 0.5 )
                { // 不要&& 会如何呢？
                    tree->root.push_back(data[i]);
                }

                else
                {
                    if (data[i].x < splitMidValue)
                        subSet1.push_back(data[i]);
                    else
                        subSet2.push_back(data[i]);
                }
            }

            if (splitAttribute == 1)
            {
                if (abs(splitMidValue - data[i].y) < 0.5 )
                {
                    tree->root.push_back(data[i]);
                }

                else
                {
                    if (data[i].y < splitMidValue)
                        subSet1.push_back(data[i]);
                    else
                        subSet2.push_back(data[i]);
                }
            }
        }

        tree->leftChild = new KdTree();
        tree->leftChild->parent = tree;
        tree->rightChild = new KdTree();
        tree->rightChild->parent = tree;
        BuildKdTree(tree->leftChild, subSet1, depth + 1);
        BuildKdTree(tree->rightChild, subSet2, depth + 1);
    }

    void myKdTree::GetKdTree(const unsigned depth)
    {
        BuildKdTree(tree, data, depth);
        cout << "The KdTree has been built!!!" << endl;
    }

    myKdTree::Position myKdTree::searchNearestPoint(const myKdTree::Position &goal)
    {
        int k = 2;
        int d = 0;
        KdTree *currentTree = tree;
        Position currentNearest = currentTree->root[0];
        while (!currentTree->isLeaf())
        {
            int index = d % k;
            if (index == 0)
            {
                if (currentTree->rightChild->isEmpty() || goal.x < currentNearest.x)
                {
                    currentTree = currentTree->leftChild;
                }
                else
                {
                    currentTree = currentTree->rightChild;
                }
            }
            if (index == 1)
            {

                if (currentTree->rightChild->isEmpty() || goal.y < currentNearest.y)
                {
                    currentTree = currentTree->leftChild;
                }
                else
                {
                    currentTree = currentTree->rightChild;
                }
            }
            d++;
        }

        currentNearest = currentTree->root[0];
        float currentDistance = measureDistance(goal, currentNearest);

        KdTree *searchDistrict;

        if (currentTree->isLeft())
        {
            if (currentTree->parent->rightChild == NULL)
                searchDistrict = currentTree;
            else
                searchDistrict = currentTree->parent->rightChild; // 不为空时，搜索另外一半的区域。
        }
        else
        {
            if (currentTree->parent->leftChild == NULL)
                searchDistrict = currentTree;
            else
                searchDistrict = currentTree->parent->leftChild;
        }

        while (searchDistrict->parent != NULL)
        {
            int index = (d + 1) % k;
            float districtDistance;
            if (index == 0)
            {
                districtDistance = abs(goal.x - searchDistrict->parent->root[0].x);
            }
            if (index == 1)
            {
                districtDistance = abs(goal.y - searchDistrict->parent->root[0].y);
            }

            if (districtDistance < currentDistance)
            { // 区域的父结点，只有这个地方需要更新当前的树为父节点，其他时候都是为搜索区域本身。更新最近距离与
                float parentDistance = measureDistance(goal, searchDistrict->parent->root[0]);

                if (parentDistance < currentDistance)
                {
                    currentDistance = parentDistance;
                    currentTree = searchDistrict->parent;
                    currentNearest = currentTree->root[0];
                }

                if (!searchDistrict->isEmpty())
                {
                    float rootDistance = measureDistance(goal, searchDistrict->root[0]); // 区域本身

                    if (rootDistance < currentDistance)
                    {
                        currentDistance = rootDistance;
                        currentTree = searchDistrict;
                        currentNearest = currentTree->root[0];
                    }
                }

                if (searchDistrict->leftChild != NULL)
                {
                    float leftDistance = measureDistance(goal, searchDistrict->leftChild->root[0]);
                    if (leftDistance < currentDistance)
                    {
                        currentDistance = leftDistance;
                        currentTree = searchDistrict->leftChild; // 原代码没有更新为子结点。
                        currentNearest = currentTree->root[0];
                    }
                }

                if (searchDistrict->rightChild != NULL)
                {
                    float rightDistance = measureDistance(goal, searchDistrict->rightChild->root[0]);
                    if (rightDistance < currentDistance)
                    {
                        currentDistance = rightDistance;
                        currentTree = searchDistrict->rightChild;
                        currentNearest = currentTree->root[0];
                    }
                }

            } // end if

            if (searchDistrict->parent->parent != NULL)
            {
                searchDistrict = searchDistrict->parent->isLeft() ? searchDistrict->parent->parent->rightChild : searchDistrict->parent->parent->leftChild;
                // 搜索区域不能到达parent到过的结点。
            }
            else
            {
                searchDistrict = searchDistrict->parent;
            }

            d++; // 别忘了++
        }

        return currentNearest;
    }

    myKdTree::~myKdTree()
    {
        DestroyTree(tree);
        cout << "KdTree has been destroyed!!!" << endl;
    }
} // namespace lidar_localization