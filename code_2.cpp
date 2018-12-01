
#include <stdio.h>
#include <string.h>
#include <fstream>
#include <iostream>
#include <queue>
#include <vector>
#include <set>
#include <math.h>
#include <random>

#define _DONOTPRINT //return
#define NOprobe 2
//#define EXPLR_BFS_MAXLEN 4
#define log_read_info //printf
#define log_shortest_path printf
#define log_pick_dir printf
#define log_path_info printf
#define log_output printf
#define log_progress printf
#define log_set_op printf

using namespace std;

enum {UP = 0, DOWN = 1, LEFT = 2, RIGHT = 3};
enum {UPi = 0, DOWNi = 1, LEFTi = 2, RIGHTi = 3};

short _DAYE[4] = {-1, 1, 0, 0};
short _DJAY[4] = {0, 0, -1, 1};
char _DIRNAME[4][6] = {"UP", "DOWN", "LEFT", "RIGHT"};


char ifileDir[50];
char ofileDir[50];

ifstream ifile;
ofstream ofile;


template<class T>
struct Vector2D
{
    T aye, jay;
    Vector2D(T i, T j) : aye(i), jay(j) {}
    Vector2D() : aye(-1), jay(-1) {}
};

struct PathInfo
{
    int explore_count = 0;
    int rating = 0;
    int id = -1;
    unsigned short serial_number = 0;
    char exit_dir = 0;
    int step_limit = 1;

    PathInfo () {}
    void Reset() {
        explore_count = 0;
        rating = 0;
        id = -1;
        serial_number = 0;
        exit_dir = 0;
        step_limit = 1;
    }
    void Reset(int _id, unsigned short srno, int step_lim) {
        explore_count = 0;
        rating = 0;
        id = _id;
        serial_number = srno;
        exit_dir = 0;
        step_limit = step_lim;
    }
};

struct BFS_pos
{
    short first, second;
    vector<char> path;
    BFS_pos(short f, short s, vector<char> p) : first(f), second(s), path(p) {}
    BFS_pos(short f, short s, char c) : first(f), second(s) {
        path.push_back(c);
    }
    BFS_pos(short f, short s, vector<char> p, char c) : first(f), second(s), path(p) {
        path.push_back(c);
    }
};

struct Tile
{
    bool isWall;
    bool isSaddle;
    bool isPlug;
    bool visited;
    unsigned int potential;
    int baseRating;
    short posAye;
    short posJay;
    int probe_id; // delete later
    bool probeTrail[NOprobe] = {0};
    unsigned short serial_number;

    void Initialize(bool is_wall, short aye, short jay) {
        isWall = is_wall;
        isSaddle = false;
        isPlug = false;
        visited = false;
        potential = (unsigned int)-1;
        baseRating = 0;
        posAye = aye;
        posJay = jay;
        probe_id = 0;
        serial_number = 0;
    }
    void Initialize(bool is_wall, bool is_Plug, short aye, short jay) {
        Initialize(is_wall, aye, jay);
        isPlug = is_Plug;
    }
};


class OfflineCoverage
{
public:
    OfflineCoverage(int height, int width, int batteryCapacity);
    ~OfflineCoverage();
    void ReadMap();
    void CalcShortestPath();
    void CalcShortestPath_adj();
    void CalcDist();
    void Output();
    void TestSet();

    void PrintMap_Raw();
    void PrintMap_Dist();
    void PrintMap_Dist_adj();
    void PrintMap_Equipotential();
    void Print_dist_vet();
    void PrintMap_Status(short curr_aye, short curr_jay);
    void PrintMap_Status_Path(int id);

    char GetRand(int id) {
        randarr[id] = (randarr[id] << 1) + ((randarr[id] & (1 << 7)) ? 1 : 0);
        return randarr[id];
    }

private:
    const unsigned int INF = (unsigned int)-1;
    const char _WALL = 'W';
    const char _FLOOR = '.';
    const char _PLUG = 'R';
    const char _SADDLE = '^';
    const char _SPLIT = '*';

    int EXPLR_BFS_MAXLEN = 50500; // absolute maximum distance in 1000x1000 map. 
    // Guarantees that it'll never finish.

    int height, width, batteryCapacity;
    Tile ** room;
    int ** pot_map[4];
    vector<char> route;
    vector< set<unsigned int> > dist_vec;


    short startAye, startJay;
    int extend_length;
    int floor_count = 0, cleaned_count = 0;
    char exit_dir = 1+2+4+8;
    int probe_id;
    unsigned short serial_number;
    int maxTiles = 0;
    int max_tile_distance = 0;

    unsigned char randarr[NOprobe];
    unsigned char mask[NOprobe];

    set<unsigned int>::iterator _set_iter;
    int _max_unvisited_dist;

    int trans_int(short aye, short jay) { return (aye << 10) + jay; }
    inline short getAye_dv(const set<unsigned int>::iterator pos) { return (*pos) >> 10; }
    inline short getJay_dv(const set<unsigned int>::iterator pos) { return (*pos) & 1023; }

    set<unsigned int>::iterator GetNextPoint();
    void RemovePoint(short aye, short jay, int potential);

    char BFSPick(vector<char>& travelOrder, short& aye, short& jay, PathInfo& pinfo);
    char Return_Pick(vector<char>& travelOrder, short& aye, short& jay, PathInfo& pinfo);
    inline char isValidPosition(short aye, short jay, bool allowWall);
    inline int RateTile_out(short aye, short jay, int pi);
    inline void RateBase(short aye, short jay, int& rating);
};



int main(int argc, char** argv) {
    int height, width, batteryCapacity;
    /*
    set<int> asdf;
    asdf.insert(5);
    asdf.insert(10);
    auto f = asdf.find(5);
    if (f != asdf.cend())
        cout << *f << endl;
    for(auto it = asdf.begin(); it!=asdf.end(); it++) {
        cout << *it << endl;
    }
    */

    if(argc <= 1) return 0xAAAA;

    // set directory
    strcat(ifileDir, argv[1]);
    strcat(ifileDir, "/floor.data");
    strcat(ofileDir, argv[1]);
    strcat(ofileDir, "/final.path");

    // open file
    //ifile.open(ifileDir, ios::in);
    ofile.open(ofileDir, ios::out);

    freopen(ifileDir, "r", stdin);

    /*
    ifile >> height >> width >> batteryCapacity;
    ifile.get();
    cout << "Input: " << height << " " << width << " " << batteryCapacity << "\n";
    */
    scanf("%d %d %d", &height, &width, &batteryCapacity);
    getchar();

    OfflineCoverage oc(height, width, batteryCapacity);
    oc.ReadMap();
    log_progress("Read map done.\n");
    oc.PrintMap_Raw();
    oc.CalcShortestPath(); // calculate potential
    log_progress("Shortest path done.\n");
    oc.Print_dist_vet();
    //oc.CalcShortestPath_adj();
    oc.PrintMap_Dist();
    oc.TestSet();
    //oc.PrintMap_Dist_adj();
    //oc.PrintMap_Equipotential();
    oc.CalcDist();
    log_progress("Calc done.\n");
    oc.Output();
    log_progress("Output done.\n");
    // close file
    //ifile.close();
    ofile.close();


    return 0;
}



OfflineCoverage::OfflineCoverage(int height, int width, int batteryCapacity) :
    height(height), width(width), batteryCapacity(batteryCapacity)
{
    EXPLR_BFS_MAXLEN = (batteryCapacity + 0.5) / 2.0;

    extend_length = batteryCapacity / 2;
    floor_count = cleaned_count = 0;
    serial_number = 0;

    room = new Tile*[height];

    for(int aye=0; aye<height; aye++) {
        room[aye] = new Tile[width];
    }

    probe_id = 0;

    // random movement
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dist(0, 255);
    for(int i=0; i<NOprobe; i++) {
        randarr[i] = dist(gen);
        mask[i] = 1;
    }

}

OfflineCoverage::~OfflineCoverage() {
    for(int aye=0; aye<height; aye++) {
        delete[] room[aye];
    }
    delete[] room;
}

void OfflineCoverage::ReadMap() {
    char c;

    for(short aye=0; aye<height; aye++) {
        log_read_info("<read>  row #%d\n", aye);
        for(short jay=0; jay<width; jay++) {
            //ifile.get(c);
            //ifile.get();
            c = getchar(); // 1/0
            getchar(); // space or new line

            log_read_info("> (%d, %d) %c... ", aye, jay, c);

            if (c == '1') { // is wall
                log_read_info("is wall.\n");
                room[aye][jay].Initialize(true, aye, jay);
            } else if (c == '0') {
                log_read_info("is floor.\n");
                room[aye][jay].Initialize(false, aye, jay);
                floor_count ++;
            } else if (c == 'R') {
                log_read_info("is plug.\n");
                room[aye][jay].Initialize(true, true, aye, jay); // set plug as wall
                room[aye][jay].visited = true;
                startAye = aye;
                startJay = jay;
            }
            else {
                log_read_info("    WTF.\n");
            }

        }

        log_read_info(" > ");
        for(short jay=0; jay<width; jay++) {
            if (room[aye][jay].isWall && !room[aye][jay].isPlug) { // is wall
                log_read_info("%c ", _WALL);
            } else if (!room[aye][jay].isWall) {
                log_read_info("%c ", _FLOOR);
            } else if (room[aye][jay].isPlug) {
                log_read_info("%c ", _PLUG);
            }
        }
        log_read_info("<\n");
    }
}

void OfflineCoverage::CalcShortestPath() { // calculate potential
    queue< pair<short, short> > visitQueue;
    pair<short, short> curr;

    log_shortest_path("[CalcShortestPath]\n");

    room[startAye][startJay].potential = 0;
    visitQueue.push( {startAye, startJay} );
    dist_vec.push_back(set<unsigned int>());
    dist_vec[0].insert(trans_int(startAye, startJay));

    // BFS
    while(!visitQueue.empty()) {
        if (dist_vec.size()-1 <= max_tile_distance) {
            dist_vec.push_back(set<unsigned int>());
        }

        log_shortest_path("\tQueue size: %lu\n", visitQueue.size());
        curr = visitQueue.front();
        visitQueue.pop();
        // count tiles at max distance
        if (room[curr.first][curr.second].potential >= batteryCapacity / 2) {
            maxTiles ++;
        }

        log_shortest_path("\t  @ %d, %d\n", curr.first, curr.second);
        for(int i=0; i<4; i++) {
            if (isValidPosition(curr.first+_DAYE[i], curr.second+_DJAY[i], false)) {
                if (room[curr.first+_DAYE[i]][curr.second+_DJAY[i]].potential == INF) {
                    
                    room[curr.first+_DAYE[i]][curr.second+_DJAY[i]].potential = room[curr.first][curr.second].potential + 1;
                    visitQueue.push( pair<short, short>(curr.first+_DAYE[i], curr.second+_DJAY[i]) );
                    
                    if (max_tile_distance < room[curr.first][curr.second].potential + 1) {
                        max_tile_distance = room[curr.first][curr.second].potential + 1;
                    }
                    dist_vec[room[curr.first][curr.second].potential + 1].insert(trans_int(curr.first+_DAYE[i], curr.second+_DJAY[i]));

                } else {
                    log_shortest_path("\t    %5s visited.\n", _DIRNAME[i]);
                }
            } else {
                log_shortest_path("\t    %5s invalid.\n", _DIRNAME[i]);
            }
        }
    }
    dist_vec.pop_back();
    _max_unvisited_dist = max_tile_distance+1;
    _set_iter = dist_vec[_max_unvisited_dist-1].begin();

    EXPLR_BFS_MAXLEN = (max_tile_distance * 2 + 2 > batteryCapacity) ? batteryCapacity/2 : max_tile_distance + 1;
    printf("max_tile_distance: %d, EXPLR_BFS_MAXLEN: %d\n", max_tile_distance, EXPLR_BFS_MAXLEN);
}

void OfflineCoverage::CalcShortestPath_adj() { // calculate potential
    log_shortest_path("[CalcShortestPath - adjacent]\n");

    for(int i=0; i<4; i++) {
        queue< pair<short, short> > visitQueue;
        pair<short, short> curr;
        if (isValidPosition(startAye+_DAYE[i], startJay+_DJAY[i], false)) {
            log_shortest_path("<CalcShortestPath - %s>\n", _DIRNAME[i]);

            bool visited[height][width];
            memset(visited, false, sizeof(bool)*height*width);

            pot_map[i] = new int*[height];

            for (int aye=0; aye<height; aye++) {
                pot_map[i][aye] = new int[width];
            }

            pot_map[i][startAye+_DAYE[i]][startJay+_DJAY[i]] = 1;
            visited[startAye+_DAYE[i]][startJay+_DJAY[i]] = true;
            visitQueue.push( pair<short, short>(startAye+_DAYE[i], startJay+_DJAY[i]) );

            // BFS
            while(!visitQueue.empty()) {
                log_shortest_path("\tQueue size: %lu\n", visitQueue.size());
                curr = visitQueue.front();
                visitQueue.pop();
                // count tiles at max distance
                if (pot_map[i][curr.first][curr.second] >= batteryCapacity / 2) {
                    maxTiles ++;
                }

                log_shortest_path("\t  @ %d, %d\n", curr.first, curr.second);
                for(int j=0; j<4; j++) {
                    if (isValidPosition(curr.first+_DAYE[j], curr.second+_DJAY[j], false)) {
                        if (!visited[curr.first+_DAYE[j]][curr.second+_DJAY[j]]) {
                            log_shortest_path("\t    %5s unvisited. Pushed into queue.  ", _DIRNAME[j]);
                            pot_map[i][curr.first+_DAYE[j]][curr.second+_DJAY[j]] = pot_map[i][curr.first][curr.second] + 1;
                            log_shortest_path("* ");
                            visitQueue.push( pair<short, short>(curr.first+_DAYE[j], curr.second+_DJAY[j]) );
                            log_shortest_path("* ");
                            visited[curr.first+_DAYE[j]][curr.second+_DJAY[j]] = true;
                            log_shortest_path("*\n");
                        } else {
                            log_shortest_path("\t    %5s visited.\n", _DIRNAME[j]);
                        }
                    } else {
                        log_shortest_path("\t    %5s invalid.\n", _DIRNAME[j]);
                    }
                }
            }
        } else {
            pot_map[i] = NULL;
        }
    }
}


void OfflineCoverage::Output() {
    if (cleaned_count != floor_count) {
        log_output("<output>  WARNING!  Floor is not fully covered!  (%d missed)\n", floor_count - cleaned_count);
    }
    log_output("<output> Total steps: ");
    ofile << route.size() << "\n";
    //printf("%lu\n", route.size());

    for(int i=0; i<route.size(); i++) {
        if(route[i] == UP) {
            startAye --;
        } else if(route[i] == DOWN) {
            startAye ++;
        } else if(route[i] == LEFT) {
            startJay --;
        } else if(route[i] == RIGHT) {
            startJay ++;
        }
        ofile << startAye << " " << startJay << "\n";
        //printf("%d %d\n", startAye, startJay);
    }
}


void OfflineCoverage::CalcDist() {


}


inline char OfflineCoverage::isValidPosition(short aye, short jay, bool allowWall) {
    return (aye >= 0 && jay >= 0 && aye <= height-1 && jay <= width-1 && !(!allowWall && room[aye][jay].isWall));
}


void OfflineCoverage::PrintMap_Raw() {
    _DONOTPRINT;
    printf("\n[RAW MAP]\n\tHeight: %d, Width: %d, Battery: %d\n", height, width, batteryCapacity);

    for(short aye=0; aye<height; aye++) {
        for(short jay=0; jay<width; jay++) {
            if (room[aye][jay].isPlug) {
                printf("%c ", _PLUG);
            } else if (room[aye][jay].isWall) {
                printf("%c ", _WALL);
            } else {
                printf("%c ", _FLOOR);
            }
        }
        printf("\n");
    }
}

void OfflineCoverage::PrintMap_Dist() {
    _DONOTPRINT;
    printf("\n[POTENTIAL MAP]\n\tHeight: %d, Width: %d, Battery: %d\n", height, width, batteryCapacity);
    for(short aye=0; aye<height; aye++) {
        for(short jay=0; jay<width; jay++) {
            if (room[aye][jay].isPlug) {
                printf("   %c", _PLUG);
            } else if (room[aye][jay].isWall) {
                printf("   %c", _WALL);
            } else {
                printf("%4u", room[aye][jay].potential);
            }
        }
        printf("\n\n");
    }
}

void OfflineCoverage::PrintMap_Dist_adj() {
    _DONOTPRINT;
    for(int i=0; i<4; i++) {
        if (pot_map[i] == NULL) {
            continue;
        }
        printf("\n[POTENTIAL MAP (%s)]\n\tHeight: %d, Width: %d, Battery: %d\n", _DIRNAME[i], height, width, batteryCapacity);
        for(short aye=0; aye<height; aye++) {
            for(short jay=0; jay<width; jay++) {
                if (room[aye][jay].isPlug) {
                    printf("   %c", _PLUG);
                } else if (room[aye][jay].isWall) {
                    printf("   %c", _WALL);
                } else {
                    printf("%4u", pot_map[i][aye][jay]);
                }
            }
            printf("\n\n");
        }
    }
}

void OfflineCoverage::PrintMap_Equipotential() {
    _DONOTPRINT;
    printf("\n[EQUIPOTENTIAL]\n\tHeight: %d, Width: %d, Battery: %d\n", height, width, batteryCapacity);

    for(short aye=0; aye<height; aye++) {
        for(short jay=0; jay<width; jay++) {
            if (room[aye][jay].isPlug) {
                printf("%c ", _PLUG);
            } else if (room[aye][jay].isWall) {
                printf("%c ", _WALL);
            } else {
                if (room[aye][jay].potential % 3 == 0)
                    printf("%c ", '~');
                else
                    printf("  ");
            }
        }
        printf("\n");
    }
}

void OfflineCoverage::PrintMap_Status(short curr_aye, short curr_jay) {
    _DONOTPRINT;
    printf("\n[MAP STATUS]\n\tFloor count %d  Cleaned count %d\n",floor_count, cleaned_count);

    for(short aye=0; aye<height; aye++) {
        for(short jay=0; jay<width; jay++) {
            if (room[aye][jay].isPlug) {
                printf("%c ", _PLUG);
            } else if (room[aye][jay].isWall) {
                printf("%c ", _WALL);
            } else {
                if (aye == curr_aye && jay == curr_jay) {
                    printf("x ");
                } else if (room[aye][jay].visited) {
                    printf("  ");
                } else {
                    printf("%c ", _FLOOR);
                }
            }
        }
        printf("\n");
    }
}

void OfflineCoverage::Print_dist_vet() {
    _DONOTPRINT;
    printf("[DIST_VEC]  max dist: %lu\n", dist_vec.size()-1);
    for(int i=0; i<dist_vec.size(); i++) {
        cout << "\t" << i << " steps away:" << "\n";
        for(auto it = dist_vec[i].begin(); it!=dist_vec[i].end(); it++) {
            cout << "\t  " << getAye_dv(it) << " " << getJay_dv(it) << "\n";
        }
    }
}

set<unsigned int>::iterator OfflineCoverage::GetNextPoint() {
    log_set_op("<set - Get>  GetNextPoint (%d)...\n", _max_unvisited_dist);
    
    if (dist_vec[_max_unvisited_dist].size() == 0) {
        log_set_op("\tPoints with max distance (%d) all visited.  Lower max.\n", _max_unvisited_dist);
        
        while(dist_vec[--_max_unvisited_dist].size() == 0) {
            log_set_op("\t  dist_vec[%d] is empty as well.  Lower!.\n", _max_unvisited_dist);
        }

        log_set_op("\t  Done.  dist_vec[%d] has %lu entry.\n", _max_unvisited_dist, dist_vec[_max_unvisited_dist].size());
        _set_iter = dist_vec[_max_unvisited_dist].begin();
    }

    if (_set_iter == dist_vec[_max_unvisited_dist].end()) {
        log_set_op("\tReached end for max distance(%d)\n", _max_unvisited_dist);
        _set_iter = dist_vec[_max_unvisited_dist].begin();
    }
    
    return _set_iter++;
}

void OfflineCoverage::RemovePoint(short aye, short jay, int potential) {
    
    log_set_op("<set - Remove>  Removing %d, %d from dist_vec[%d]\n", aye, jay, potential);
    set<unsigned int>::iterator iter = dist_vec[potential].find(trans_int(aye, jay));
    if (iter == dist_vec[potential].cend()) {
        perror("\tNot found in dist_vec!\n");
    } else {
        _set_iter = dist_vec[potential].erase(iter);
        printf("\t  dist_vec[%d].size(): %lu\n", potential, dist_vec[potential].size());
        if (_set_iter != dist_vec[potential].cend()) {
            //cout << "..." << getAye_dv(_set_iter) << " " << getJay_dv(_set_iter) << "\n";
        }
        else {
            cout << "it's the end...\n";
        }
    }
    
}

void OfflineCoverage::TestSet() {
    for(int i=0; i<floor_count; i++) {
        set<unsigned int>::iterator val = GetNextPoint();
        printf("(pot: %d) %d, %d\n", room[getAye_dv(val)][getJay_dv(val)].potential, getAye_dv(val), getJay_dv(val));
        RemovePoint(getAye_dv(val), getJay_dv(val), room[getAye_dv(val)][getJay_dv(val)].potential);
    }
}