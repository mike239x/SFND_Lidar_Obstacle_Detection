/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insert(std::vector<float> point, int id)
	{
		// DONE: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
        if (root == nullptr)
        {
            root = new Node(point, id);
        }
        else
        {
            Node* cur = root;
            Node* nxt = cur;
            bool vert = true; // split vertically
            while (true)
            {
                bool left;
                if (vert)
                    if (point[0] <= cur->point[0])
                    {
                        nxt = cur->left;
                        left = true;
                    }
                    else
                    {
                        nxt = cur->right;
                        left = false;
                    }
                else
                    if (point[1] <= cur->point[1])
                    {
                        nxt = cur->left;
                        left = true;
                    }
                    else
                    {
                        nxt = cur->right;
                        left = false;
                    }
                if (nxt == nullptr)
                {
                    if (left)
                        cur->left = new Node(point, id);
                    else
                        cur->right = new Node(point, id);
                    break;
                }
                cur = nxt;
                vert = !vert;
            }
        }
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
        auto dist = [](std::vector<float> a, std::vector<float> b) -> float
        {
            float d = 0.0;
            for (size_t i = 0; i < a.size(); ++i)
                d += (a[i]-b[i])*(a[i]-b[i]);
            return std::sqrt(d);
        };
		std::vector<int> ids;
        std::vector<Node*> branches;
        std::vector<int> split_types;
        branches.push_back(root);
        split_types.push_back(0);
        while (branches.size())
        {
            Node* node = branches.back();
            branches.pop_back();
            int split_type = split_types.back();
            split_types.pop_back();
            {
                float d = dist(node->point, target);
                if (d < distanceTol)
                    ids.push_back(node->id);
            }
            {
                float d = node->point[split_type] - target[split_type];
                // positive d means target is to the left of the current point
                int next_split_type = (split_type + 1) % target.size();
                if (d < distanceTol && node->right != nullptr)
                {
                    // add right branch to the search
                    branches.push_back(node->right);
                    split_types.push_back(next_split_type);
                }
                if (d > -distanceTol && node->left != nullptr)
                {
                    // add left branch to the search
                    branches.push_back(node->left);
                    split_types.push_back(next_split_type);
                }
            }
        }
		return ids;
	}
	

};

