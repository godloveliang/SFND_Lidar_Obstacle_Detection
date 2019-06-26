
// Structure to represent node of kd tree
struct Node
{
	pcl::PointXYZI point;
	int id;
	Node* left;
	Node* right;

	Node(pcl::PointXYZI arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	//关于双重指针的理解 ：https://blog.csdn.net/qq_33631303/article/details/78007413
	//指向指针的指针，通过改变指针存放的地址来改变指向。
	void inserthelper(Node** node, uint depth, pcl::PointXYZI point, int id)
	{
		if(*node==NULL)
			*node = new Node(point, id);
		else
		{
			//uint cd = depth % 3;
			uint cd = depth % 2;
			if(cd == 0)
			{
				if(point.x < ((*node)->point.x))
					inserthelper(&((*node)->left), depth+1, point, id);
				else
					inserthelper(&((*node)->right), depth+1, point, id);	
			}
			else
			{
				if(point.y < ((*node)->point.y))
					inserthelper(&((*node)->left), depth+1, point, id);
				else
					inserthelper(&((*node)->right), depth+1, point, id);	
			}	
		}	
	}

	void insert(pcl::PointXYZI point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		inserthelper(&root, 0, point, id);
	}

	void searchhelp(pcl::PointXYZI target, Node* node, int depth, float distanceTol, std::vector<int> &ids)
	{
		if(node!=NULL)
		{
			if( (node->point.x >= (target.x - distanceTol)) && (node->point.x <= (target.x + distanceTol)) \
				&& (node->point.y >= (target.y - distanceTol)) && (node->point.y <= (target.y + distanceTol)))
			{
				float dist = sqrt(pow((node->point.x - target.x), 2) + pow((node->point.y - target.y), 2));
				if(dist<=distanceTol)
					ids.push_back(node->id);
			}

			if(depth%2 == 0)
			{
				if((target.x - distanceTol) < node->point.x)
					searchhelp(target, node->left, depth+1, distanceTol, ids);
				if((target.x + distanceTol) > node->point.x)
					searchhelp(target, node->right, depth+1, distanceTol, ids);
			}
			else
			{
				if((target.y - distanceTol) < node->point.y)
					searchhelp(target, node->left, depth+1, distanceTol, ids);
				if((target.y + distanceTol) > node->point.y)
					searchhelp(target, node->right, depth+1, distanceTol, ids);
			}
			
		}
	}
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(pcl::PointXYZI target, float distanceTol)
	{
		std::vector<int> ids;
		searchhelp(target, root, 0, distanceTol, ids);
		return ids;
	}	

};




