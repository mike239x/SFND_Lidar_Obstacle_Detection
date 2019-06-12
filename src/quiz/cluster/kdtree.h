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
		std::vector<int> ids;
		return ids;
	}
	

};




