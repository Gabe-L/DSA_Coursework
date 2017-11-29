#include <iostream>
#include <fstream>
#include <list>
#include <string>
#include <set>
#include <vector>
#include <chrono>

using namespace std;
using std::list;
using std::chrono::duration_cast;
using std::chrono::milliseconds;
using the_clock = std::chrono::steady_clock;


struct coord {
	int x, y;
};

int randomInt(int min, int max) {
	return min + (rand() % (max - min + 1));
}

struct Vertex {
	std::string name;
	bool blocked;
	int distance; //-1 if not filled in yet
	int f, g, h;
	coord coords;
	std::list<pair<Vertex*, int>> neighbours;
	Vertex* parent;

	Vertex(std::string name_, coord coord_) :
		name(name_), coords(coord_), blocked(false), distance(-1) {}
};

void connect(Vertex& a, Vertex& b, int weight) {
	a.neighbours.push_back({ &b, weight });
	b.neighbours.push_back({ &a, weight });
}


std::list<Vertex *> smallGrid(Vertex** first, Vertex** last) {

	Vertex* grid[5][5];

	std::list<Vertex *> returnGrid;

	int size = 5;

	for (int y = 0; y < size; y++) {
		for (int x = 0; x < size; x++) {
			grid[x][y] = new Vertex(std::to_string(x) + std::to_string(y), { x,y });
			returnGrid.push_front(grid[x][y]);
			if (x > 0) {
				connect(*grid[x][y], *grid[x - 1][y], 1);//left
				if (y > 0) {
					connect(*grid[x][y], *grid[x - 1][y - 1], 1.5);//upper left
				}
			}
			if (y > 0) {
				connect(*grid[x][y], *grid[x][y - 1], 1);//above
				if (x < size - 1) {
					connect(*grid[x][y], *grid[x + 1][y - 1], 1.5); //top right
				}
			}
		}
	}

	int randX = randomInt(0, 5), randY = randomInt(0, 5);
	*first = grid[randX][randY];

	randX = randomInt(0, 5);  randY = randomInt(0, 5);
	*last = grid[randX][randY];

	return returnGrid;
}

std::list<Vertex *> medGrid() {

	Vertex* grid[10][10];

	std::list<Vertex *> returnGrid;

	int size = 10;

	for (int y = 0; y < size; y++) {
		for (int x = 0; x < size; x++) {
			grid[x][y] = new Vertex(std::to_string(x) + std::to_string(y), { x,y });
			returnGrid.push_front(grid[x][y]);
			if (x > 0) {
				connect(*grid[x][y], *grid[x - 1][y], 1);//left
				if (y > 0) {
					connect(*grid[x][y], *grid[x - 1][y - 1], 1);//upper left
				}
			}
			if (y > 0) {
				connect(*grid[x][y], *grid[x][y - 1], 1);//above
				if (x < size - 1) {
					connect(*grid[x][y], *grid[x + 1][y - 1], 1); //top right
				}
			}
		}
	}
	return returnGrid;
}

std::list<Vertex *> largeGrid(Vertex** first, Vertex** last, int size) {

	std::list<Vertex*> returnGrid;

	vector<vector<Vertex*>> grid;

	grid.resize(size);
	for (auto &line : grid) {
		line.resize(size);
	}

	for (int y = 0; y < size; y++) {
		for (int x = 0; x < size; x++) {
			returnGrid.push_back(new Vertex(std::to_string(x) + ", " + std::to_string(y), { x,y }));
			grid[x][y] = returnGrid.back();

			if (x > 0) {
				connect(*grid[x][y], *grid[x - 1][y], 1);//left
				if (y > 0) {
					connect(*grid[x][y], *grid[x - 1][y - 1], 1);//upper left
				}
			}
			if (y > 0) {
				connect(*grid[x][y], *grid[x][y - 1], 1);//above
				if (x < size - 1) {
					connect(*grid[x][y], *grid[x + 1][y - 1], 1); //top right
				}
			}
		}
	}

	int randX = randomInt(0, size / 10), randY = randomInt(0, size / 10);
	*first = grid[randX][randY];

	randX = randomInt(size - (size / 10), size - 1);  randY = randomInt(size - (size / 10), size - 1);
	*last = grid[randX][randY];

	return returnGrid;
}

std::list<Vertex*> leeAlgorithm(Vertex* start, Vertex* goal) {
	std::set<Vertex*> open, close;
	start->distance = 0;

	Vertex* current;
	int new_dist;

	open.insert(start);
	
	//Phase 1
	while (!open.empty()) {
		current = *open.begin();

		if (current == goal) {
			break;
		}

		open.erase(current);
		close.insert(current);

		for (auto v : current->neighbours) {
			if (close.count(v.first) > 0) {
				if (v.first->distance > current->distance + v.second) {
					v.first->distance = current->distance + v.second;
				}
				continue;
			}

			new_dist = current->distance + v.second;

			if (open.count(v.first) < 1) {
				open.insert(v.first);
			}
			else if (new_dist >= v.first->distance) {
				continue;
			}
			v.first->distance = new_dist;
			v.first->parent = current;
		}
	}

	std::list<Vertex*> path;
	current = goal;
	Vertex* betterPath = goal;
	//Phase 2
	while (current->distance > 0) {
		path.push_front(current);
		for (auto node : current->neighbours) {
			if (node.first->distance < betterPath->distance && node.first->distance > -1) {
				betterPath = node.first;
			}
		}
		current = betterPath;
	}
	path.push_front(start);

	return path;
}

int heuristic(Vertex* node, Vertex* goal) {
	int less, D = 200, D2 = 200;
	int dx = abs(node->coords.x - goal->coords.x);
	int dy = abs(node->coords.y - goal->coords.y);

	if (dx < dy) {
		less = dx;
	}
	else {
		less = dy;
	}

	for (auto v : node->neighbours) {
		if (v.first->coords.x == node->coords.x || v.first->coords.y == node->coords.y) { //v is in direction NSEW
			if (v.second < D) {
				D = v.second;
			}
		}
		else { //v is inbetween an NSEW direction
			if (v.second < D2) {
				D2 = v.second;
			}
		}
	}

	int h = D * (dx + dy) + (D2 - 2 * D) * less;

	return h;
}

Vertex* lowestF(set<Vertex*> open) {
	Vertex* low = *open.begin();

	for (auto v : open) {
		if (v->f < low->f) {
			low = v;
		}
	}
	return low;
}

std::list<Vertex *> aStar(Vertex* start, Vertex* end) {
	std::set<Vertex*> open, close;
	start->g = 0;
	start->f = start->g + heuristic(start, end);

	/*FOR TESTING*/
	start->h = heuristic(start, end);
	/*-----------------------------*/

	Vertex* current;
	int new_g;

	open.insert(start);

	while (!open.empty()) {
		current = lowestF(open);

		if (current == end) {
			break;
		}

		open.erase(current);
		close.insert(current);

		for (auto v : current->neighbours) {
			if (close.count(v.first) > 0) {
				continue;
			}

			new_g = current->g + v.second;

			if (open.count(v.first) < 1) {
				open.insert(v.first);
			}
			else if (new_g >= v.first->g) {
				continue;
			}
			v.first->g = new_g;
			v.first->f = v.first->g + heuristic(v.first, end);
			/*FOR TESTING*/
			v.first->h = heuristic(v.first, end);
			/*---------------------------------*/
			v.first->parent = current;
		}
	}

	//tracing back

	std::list<Vertex*> path;
	Vertex* track;
	track = end;
	while (track != start) {
		path.push_front(track);
		track = track->parent;
	}
	path.push_front(start);

	return path;
}

Vertex* findVertex(coord dest, std::list<Vertex*> graph) {
	for (auto node : graph) {
		if (node->coords.x == dest.x && node->coords.y == dest.y) {
			return node;
		}
	}
	return nullptr;
}

bool isVertex(coord dest, std::list<Vertex*> graph) {
	for (auto node : graph) {
		if (node->coords.x == dest.x && node->coords.y == dest.y) {
			return true;
		}
	}
	return false;
}

void display(std::list<Vertex *>& graph) {

	int loopSize = sqrt(graph.size());

	for (int y = 0; y < loopSize; y++) {
		for (int x = 0; x < loopSize; x++) {
			Vertex* tempNode = findVertex({ x,y }, graph);
			if (tempNode != nullptr) {
				switch (tempNode->name.size())
				{
				case 1:
					cout << tempNode->name << "__|";
					break;
				case 2:
					cout << tempNode->name << "_|";
					break;
				case 3:
					cout << tempNode->name << "|";
					break;
				default:
					cout << tempNode->name;
					break;
				}
			}
			else {
				cout << "___|";
			}
		}
		cout << endl;
	}
}

std::list<Vertex*> generateGraph(int size) {
	std::list<Vertex*> graph;
	for (int i = 0; i < size; i++) {
		int randomX = (rand() % size) + 1, randomY = (rand() % size) + 1;
		coord tempCrd = { randomX, randomY };
		graph.push_back(new Vertex{ to_string(i), tempCrd });
	}

	int range = size / 5;
	for (auto node : graph) {
		coord tempCrd = node->coords;
		for (int y = tempCrd.y - range; y < tempCrd.y + range; y++) {
			for (int x = tempCrd.x - range; x < tempCrd.x + range; x++) {
				Vertex* search = findVertex({ x,y }, graph);
				if (search != nullptr && (tempCrd.x != x && tempCrd.y != y)) {
					connect(*node, *search, 2);
				}
			}
		}
	}
	return graph;
}

void storeGrid(std::list<Vertex *>& graph, std::string fileName, char dataType) {
	toupper(dataType);
	ofstream csv;
	csv.open(fileName);
	int loopSize = sqrt(graph.size());
	int track = 0;

	for (auto node : graph) {
		track++;
		if (node != nullptr) {
			switch (dataType) {
			case 'D':
				csv << node->distance << ",";
				break;
			case 'F':
				csv << node->f << ",";
				break;
			case 'G':
				csv << node->g << ",";
				break;
			default:
				csv << "error,";
			}
		}
		else {
			csv << " ,";
		}
		if (track == loopSize) {
			csv << "\n";
			track = 0;
		}

	}

	csv.close();
}

void test() {
	Vertex va("A", { 0,1 }), vb("B", { 1,0 }), vc("C", { 2,1 }), vd("D", { 1, 2 }), ve("E", { 3,0 }), vf("F", { 3,2 }), vg("G", { 2,3 }), vh("H", { 4,3 }), vi("I", { 4,1 }), vx("X", { 2,8 }), vy("Y", { 5,8 }), vz("Z", { 7,10 });
	connect(va, vb, 7);
	connect(vb, vc, 3);
	connect(vb, vd, 2);
	connect(vc, ve, 4);
	connect(vc, vf, 2);
	connect(vf, vh, 6);
	connect(vf, vi, 3);
	connect(vd, vg, 5);
	connect(vg, vf, 1);
	connect(vi, vh, 2);

	connect(vg, vx, 4);
	connect(vx, vy, 2);
	connect(vy, vz, 6);
	//vc.blocked = true;

	std::list<Vertex *> graph = generateGraph(50);
	
	//generateGraph(&graph, graph.front(), 0);
	
	va.distance = 0;
	//phase1(va, vh, 0);

	std::list<Vertex *> path;
	/*phase2(&vh, path);
	path.reverse();
	cout << "Distances:" << endl;*/
	
	Vertex* start = *graph.begin();
	Vertex* last = *graph.begin();

	//path = aStar(graph.front(), graph.back());

	storeGrid(graph, "LeeGrid", 'D');
	storeGrid(graph, "A_Grid", 'F');

	
	
	cout << endl << "Path taken:" << endl;
}


int main() {
	srand(time(NULL));

	//test();
	//small grid test
	std::list<Vertex*> aPath, lPath;

	Vertex* first = {};
	Vertex* last = {};
	the_clock::time_point startTime, endTime;
	float leeTime, aTime;

	std::list<Vertex*> small = largeGrid(&first, &last, 100);
	
	cout << "Grid generation finished..." << endl;

	//std::list<Vertex*> path = aStar(small.front(), small.back)
	//std::list<Vertex*> med = medGrid();
	//std::list<Vertex*> large = largeGrid();

	startTime = the_clock::now();
	lPath = leeAlgorithm(first, last);
	endTime = the_clock::now();

	auto totalTime = duration_cast<milliseconds>(endTime - startTime).count();
	leeTime = totalTime;

	cout << "Lee algorithm finished..." << endl;

	startTime = the_clock::now();
	aPath = aStar(first, last);
	endTime = the_clock::now();

	totalTime = duration_cast<milliseconds>(endTime - startTime).count();
	aTime = totalTime;

	cout << "A* algorithm finished..." << endl;

	storeGrid(small, "LeeGrid - Small.csv", 'D');
	storeGrid(small, "A_Grid - Small.csv", 'F');

	cout << "Grid storing finished..." << endl;

	ofstream leeFile;
	leeFile.open("Lee_Path.csv");
	for (auto node : lPath) {
		leeFile << node->distance << ",";
	}
	leeFile.close();

	ofstream aFile;
	aFile.open("A_Path.csv");
	for (auto node : aPath) {
		aFile << node->distance << ",";
	}
	aFile.close();

	ofstream leeInfo;
	leeInfo.open("Lee_Info.txt");
	
	leeInfo << "Start cell: " << first->coords.x << ", " << first->coords.y << "\n";
	leeInfo << "  End cell: " << last->coords.x << ", " << last->coords.y << "\n";
	leeInfo << "Time taken: " << leeTime << " milliseconds.";

	leeInfo.close();

	ofstream aInfo;
	aInfo.open("A_Info.txt");

	aInfo << "Start cell: " << first->coords.x << ", " << first->coords.y << "\n";
	aInfo << "  End cell: " << last->coords.x << ", " << last->coords.y << "\n";
	aInfo << "Time taken: " << aTime << " milliseconds.";

	aInfo.close();
	cout << "Extra info storage finished..." << endl;
	cout << "Finished exectuing...";

	//wait for user input to end program

	cin.get();

	return 0;

}