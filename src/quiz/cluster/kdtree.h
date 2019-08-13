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
	std::vector<int> search(const std::vector<float>& target, float distanceTol) const
	{
		std::vector<int> ids;
		search(root, target, distanceTol, 0, ids);
		return ids;
	}

	void search(const Node* node, const std::vector<float>& target, float distanceTol, unsigned int depth, std::vector<int>& ids) const
	{
		if (node == nullptr)
			return;

		if (node->point[0] >= target[0] - distanceTol &&
			node->point[0] <= target[0] + distanceTol &&
			node->point[1] >= target[1] - distanceTol &&
			node->point[1] <= target[1] + distanceTol) {

			auto d{ 0.0 };
			for (int i = 0; i < target.size(); i++)
				d += pow(target[i] - node->point[i], 2.0);

			if (sqrt(d) <= distanceTol)
				ids.push_back(node->id);
		}

		const auto cd{ depth % 2 };

		if (target[cd] - distanceTol < node->point[cd])
			search(node->left, target, distanceTol, depth + 1, ids);

		if (target[cd] + distanceTol > node->point[cd])
			search(node->right, target, distanceTol, depth + 1, ids);
	}
};




