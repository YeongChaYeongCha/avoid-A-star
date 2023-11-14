#include "stdc++.h"

constexpr int MAX = 101;
constexpr double INF = 1e9 + 7;

const int dx1[4] = { 0, 0, 1, -1 };
const int dy1[4] = { -1, 1, 0, 0 };

const int dx2[4] = { 1, -1, -1, 1 };
const int dy2[4] = { -1, 1, -1, 1 };

using Pair = pair<int, int>;
using pPair = pair<double, Pair>;

struct Cell {
	int parent_x, parent_y;
	double f, g, h;
};

char zmap[MAX][MAX];
int ROW = 0, COL = 0;

vector<pair<float, float>> obstacle;
vector<pair<float, float>> way_point;

bool isDestination(int row, int col, Pair dst) {
	if (row == dst.first && col == dst.second) return true;
	return false;
}

bool isInRange(int row, int col) {
	return (row >= 0 && row < ROW && col >= 0 && col < COL);
}

bool isUnBlocked(vector<vector<int>>& map, int row, int col) {
	return (map[row][col] == 0);
}

double GethValue(int row, int col, Pair dst) {
	return (double)sqrt(pow(row - dst.first, 2) + pow(col - dst.second, 2));
}

void tracePath(Cell cellDetails[MAX][MAX], Pair dst) {
	stack<Pair> s;
	int y = dst.first;
	int x = dst.second;

	s.push({ y, x });
	while (!(cellDetails[y][x].parent_x == x && cellDetails[y][x].parent_y == y)) {
		int tempy = cellDetails[y][x].parent_y;
		int tempx = cellDetails[y][x].parent_x;
		y = tempy;
		x = tempx;
		s.push({ y, x });
	}

	while (!s.empty()) {
		zmap[s.top().first][s.top().second] = '*';
		s.pop();
	}
}

bool aStarSearch(vector<vector<int>>& map, Pair src, Pair dst) {
	if (!isInRange(src.first, src.second) || !isInRange(dst.first, dst.second))
		return false;
	if (!isUnBlocked(map, src.first, src.second) || !isUnBlocked(map, dst.first, dst.second))
		return false;
	if (isDestination(src.first, src.second, dst))
		return false;

	bool closedList[MAX][MAX];
	memset(closedList, false, sizeof(closedList));

	Cell cellDetails[MAX][MAX];

	for (int i = 0; i < ROW; ++i) {
		for (int j = 0; j < COL; ++j) {
			cellDetails[i][j].f = cellDetails[i][j].g = cellDetails[i][j].h = INF;
			cellDetails[i][j].parent_x = cellDetails[i][j].parent_y = -1;
		}
	}

	int sy = src.first;
	int sx = src.second;
	cellDetails[sy][sx].f = cellDetails[sy][sx].g = cellDetails[sy][sx].h = 0.0;
	cellDetails[sy][sx].parent_x = sx;
	cellDetails[sy][sx].parent_y = sy;

	set<pPair> openList;
	openList.insert({ 0.0, { sy, sx } });

	while (!openList.empty()) {
		pPair p = *openList.begin();
		openList.erase(openList.begin());

		int y = p.second.first;
		int x = p.second.second;
		closedList[y][x] = true;

		double ng, nf, nh;

		for (int i = 0; i < 4; ++i) {
			int ny = y + dy1[i];
			int nx = x + dx1[i];

			if (isInRange(ny, nx)) {
				if (isDestination(ny, nx, dst)) {
					cellDetails[ny][nx].parent_y = y;
					cellDetails[ny][nx].parent_x = x;
					tracePath(cellDetails, dst);
					return true;
				}

				else if (!closedList[ny][nx] && isUnBlocked(map, ny, nx)) {
					ng = cellDetails[y][x].g + 0.5;
					nh = GethValue(ny, nx, dst);
					nf = ng + nh;

					if (cellDetails[ny][nx].f == INF || cellDetails[ny][nx].f > nf) {
						cellDetails[ny][nx].f = nf;
						cellDetails[ny][nx].g = ng;
						cellDetails[ny][nx].h = nh;
						cellDetails[ny][nx].parent_x = x;
						cellDetails[ny][nx].parent_y = y;
						openList.insert({ nf, { ny, nx } });
					}
				}
			}
		}

		for (int i = 0; i < 4; ++i) {
			int ny = y + dy2[i];
			int nx = x + dx2[i];

			if (isInRange(ny, nx)) {
				if (isDestination(ny, nx, dst)) {
					cellDetails[ny][nx].parent_y = y;
					cellDetails[ny][nx].parent_x = x;
					tracePath(cellDetails, dst);
					return true;
				}
				else if (!closedList[ny][nx] && isUnBlocked(map, ny, nx)) {
					ng = cellDetails[y][x].g + 0.707;
					nh = GethValue(ny, nx, dst);
					nf = ng + nh;

					if (cellDetails[ny][nx].f == INF || cellDetails[ny][nx].f > nf) {
						cellDetails[ny][nx].f = nf;
						cellDetails[ny][nx].g = ng;
						cellDetails[ny][nx].h = nh;
						cellDetails[ny][nx].parent_x = x;
						cellDetails[ny][nx].parent_y = y;
						openList.insert({ nf, { ny, nx } });
					}
				}
			}
		}
	}

	return false;
}

void PrintMap() {
	for (int i = 0; i < ROW; ++i) {
		for (int j = 0; j < COL; ++j) {
			cout << zmap[i][j];
		}
		cout << '\n';
	}
	cout << "------------------------------" << "\n";
	cout << "Way Point" << "\n";
	for (int i = ROW - 1; i >= 0; --i) {
		for (int j = COL - 1; j >= 0; --j) {
			if (zmap[i][j] == '*'){
				way_point.push_back({float((ROW - i - 1)/2.0), float((j - COL / 2)/2.0)});
	    		cout << "(" << (ROW - i - 1)/2.0 << ", " << (j - COL / 2)/2.0 << ")" << " | ";
			}
		}
	}
	cout << '\n';
	// cout<<"Way Point Num : "<<way_point.size()<<"\n";
}

void obstacle_pos_callback(const visualization_msgs::MarkerArray &cone_boundingbox){
	if(cone_boundingbox.markers.size()==1){
		for(auto iter=cone_boundingbox.markers.begin(); iter!=cone_boundingbox.markers.end(); iter++){
			float cone_pos_x=iter->pose.position.x;
			float cone_pos_y=iter->pose.position.y;

			cout<<cone_pos_x<<" "<<-cone_pos_y<<"\n";
			cout<<"-------------------------"<<"\n";
		
			obstacle.push_back({cone_pos_x, -cone_pos_y});
    	}
		Pair src, dst;
		int row, col;

		// cout << "Obstacle Position : ";
		float obstacle_x=obstacle.begin()->first; float obstacle_y=obstacle.begin()->second;
		cout<<"--Obstacle Position--"<<"\n";
		cout<<obstacle_x<<", "<<obstacle_y<<"\n";
		// float obstacle_x, obstacle_y;

		if (obstacle_x - floor(obstacle_x) < 0.25) obstacle_x = floor(obstacle_x);
		else if (obstacle_x - floor(obstacle_x) < 0.75) obstacle_x = floor(obstacle_x) + 0.5;
		else obstacle_x = ceil(obstacle_x);

		if (obstacle_y - floor(obstacle_y) < 0.25) obstacle_y = floor(obstacle_y);
		else if (obstacle_y - floor(obstacle_y) < 0.75) obstacle_y = floor(obstacle_y) + 0.5;
		else obstacle_y = ceil(obstacle_y);

		cout<<obstacle_x<<", "<<obstacle_y<<"\n";

		// cout << "Row and Col : ";
		//std::cin >> row >> col;
		ROW = obstacle_x / 0.5 + 7;
		COL = 13;

		vector<vector<int>> grid(ROW, vector<int>(COL));
		for (int i = 0; i < ROW; ++i) {
			for (int j = 0; j < COL; ++j) {
				grid[i][j]=0;
			}
		}
		grid[ROW - 1][COL / 2] = 2;
		grid[0][COL / 2] = 3;

		grid[int(ROW - (obstacle_x / 0.5))][int(COL / 2 + obstacle_y / 0.5)] = 1;

		for (int i = 0; i < ROW; ++i) {
			for (int j = 0; j < COL; ++j) {
				if (i >= ROW - (obstacle_x / 0.5) - 2 && i <= ROW - (obstacle_x / 0.5) + 2) {
					if (j >= COL / 2 + obstacle_y / 0.5 - 2 && j <= COL / 2 + obstacle_y / 0.5 + 2) {
						grid[i][j] = 1;
					}
				}
			}
		}

		for (int i = 0; i < ROW; ++i) {
			for (int j = 0; j < COL; ++j) {
				if (grid[i][j] == 2) {
					src = { i, j };
					grid[i][j] = 0;
				}
				if (grid[i][j] == 3) {
					dst = { i, j };
					grid[i][j] = 0;
				}
			}
		}

		for (int i = 0; i < ROW; ++i) {
			for (int j = 0; j < COL; ++j) {
				zmap[i][j] = grid[i][j] + '0';
			}
		}

		if (aStarSearch(grid, src, dst))
			PrintMap();
		else
			cout << "Fail";
	}	

	obstacle.clear();
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "A_star_path_maker");
	ros::NodeHandle nh;
	ros::Subscriber sub_obstacle_pos=nh.subscribe("/object_bounding_boxes", 100, obstacle_pos_callback);
	ros::Publisher pub_way_point=nh.advertise<visualization_msgs::MarkerArray>("/way_points", 1000);

	ros::Rate loop_rate(10);
	visualization_msgs::MarkerArray way_point_array;

	while (ros::ok()) {
        for (auto iter = way_point.begin(); iter != way_point.end(); iter++) {
            visualization_msgs::Marker way_point_marker;
            way_point_marker.header.frame_id = "velodyne";
            way_point_marker.header.stamp = ros::Time::now();
            way_point_marker.ns = "cylinder";
            way_point_marker.id = std::distance(way_point.begin(), iter); // Use distance as a unique ID
            way_point_marker.type = 3;
            way_point_marker.action = visualization_msgs::Marker::ADD;
            way_point_marker.pose.position.x = iter->first;
            way_point_marker.pose.position.y = -iter->second;
            way_point_marker.pose.position.z = 0.0;
            way_point_marker.pose.orientation.x = 0.0;
            way_point_marker.pose.orientation.y = 0.0;
            way_point_marker.pose.orientation.z = 0.0;
            way_point_marker.pose.orientation.w = 1.0;
            way_point_marker.scale.x = 0.2;
            way_point_marker.scale.y = 0.2;
            way_point_marker.scale.z = 0.2;
            way_point_marker.color.a = 1.0;
            way_point_marker.color.r = 0.0;
            way_point_marker.color.g = 1.0;
            way_point_marker.color.b = 0.0;
            way_point_marker.lifetime = ros::Duration(0.2);

            way_point_array.markers.push_back(way_point_marker);
        }

        pub_way_point.publish(way_point_array);
        way_point_array.markers.clear();
        way_point.clear();

		ros::spinOnce();
		loop_rate.sleep();
	}


	return 0;
}