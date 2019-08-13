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
	Node* root{ nullptr };

	KdTree() = default;

	void insert(const std::vector<float> point, int id)
	{
		insert(root, point, id);
	}

	void insert(Node*& node, const std::vector<float> point, int id, unsigned int depth = 0)
	{
		if (node == nullptr)
		{
			node = new Node{ point, id };
		}
		else {
			const auto cd{ depth % 2 };

			if (point[cd] < node->point[cd])
				insert(node->left, point, id, depth + 1);
			else
				insert(node->right, point, id, depth + 1);
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		return ids;
	}
	

};




