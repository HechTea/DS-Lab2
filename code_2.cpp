
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
#define log_shortest_path //printf
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
    BFS_pos(short f, short s) : first(f), second(s) {}
    BFS_pos(short f, short s, vector<char> p) : first(f), second(s), path(p) {}
    BFS_pos(short f, short s, char c) : first(f), second(s) {
        path.push_back(c);
    }
    BFS_pos(short f, short s, vector<char> p, char c) : first(f), second(s), path(p) {
        path.push_back(c);
    }
};

struct BFS_mini_map
{
    BFS_mini_map(int maxLen, short caye, short cjay) : _caye(caye), _cjay(cjay), mcaye(maxLen), mcjay(maxLen) {
        _visited = new bool*[maxLen*2+1];
        for(int i=0; i < maxLen * 2 + 1; i++) {
            _visited[i] = new bool[maxLen*2+1];
            memset(_visited[i], false, sizeof(bool)*2+1);
        }
        _visited[mcaye][mcjay] = true;
    }
    ~BFS_mini_map() {
        for(int i=0; i < mcaye * 2 - 1; i++) {
            delete[] _visited[i];
        }
        delete[] _visited;
    }

    bool isChecked(short aye, short jay) {
        return _visited[mcaye + (aye - _caye)][mcjay + (jay - _cjay)];
    }
    void Check(short aye, short jay) {
        _visited[mcaye + (aye - _caye)][mcjay + (jay - _cjay)] = true;
    }

    bool ** _visited;
    short _caye, _cjay;
    short mcaye, mcjay;
    short 
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
    void Print_dir_dist_vet();
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
    int mini_maxLen = 5;

    int height, width, batteryCapacity;
    vector<char> route;
    Tile ** room;
    int ** pot_map[4];
    vector< set<unsigned int> > dist_vec;
    vector< set<unsigned int> > dir_dist_vec[4];


    short startAye, startJay;
    int extend_length;
    int floor_count = 0, cleaned_count = 0;
    char exit_dir = 1+2+4+8;
    int probe_id;
    unsigned short serial_number;
    int maxTiles = 0;
    int max_tile_distance = 0;
    int dir_max_tile_distance[4] = {0};

    unsigned char randarr[NOprobe];
    unsigned char mask[NOprobe];

    set<unsigned int>::iterator _set_iter;
    set<unsigned int>::iterator _dir_set_iter[4];
    int _max_unvisited_dist;
    int _dir_max_unvisited_dist[4];

    int trans_int(short aye, short jay) { return (aye << 10) + jay; }
    inline short getAye_dv(const set<unsigned int>::iterator pos) { return (*pos) >> 10; }
    inline short getJay_dv(const set<unsigned int>::iterator pos) { return (*pos) & 1023; }

    set<unsigned int>::iterator GetNextPoint();
    set<unsigned int>::iterator GetNextPoint(int dir);
    void RemovePoint(short aye, short jay, int potential);
    void RemovePoint(short aye, short jay, int potential, int dir);

    char MaxDistOut(vector<char>& travelOrder, short& aye, short& jay, PathInfo& pinfo);
    char Wander(vector<char>& travelOrder, short& aye, short& jay, PathInfo& pinfo);
    char ReturnRecharge(vector<char>& travelOrder, short& aye, short& jay, PathInfo& pinfo);
    char PosToPlug(vector<char>& travelOrder, short& aye, short& jay, PathInfo& pinfo);
    char ChangeExitDir(vector<char>& travelOrder, short& aye, short& jay, PathInfo& pinfo);
    char BFSProbe(vector<char>& travelOrder, short& aye, short& jay, const PathInfo& pinfo);
    inline char CountAdjacentUnvisited(short& aye, short& jay);

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
    oc.Print_dir_dist_vet();
    oc.CalcShortestPath_adj();
    oc.PrintMap_Dist();
    oc.PrintMap_Dist_adj();
    oc.TestSet();
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

/// nope
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

            dir_dist_vec[i].push_back(set<unsigned int>());
            dir_dist_vec[i][0].insert(trans_int(startAye, startJay));
            dir_dist_vec[i].push_back(set<unsigned int>());
            dir_dist_vec[i][1].insert(trans_int(startAye+_DAYE[i], startJay+_DJAY[i]));

            pot_map[i][startAye+_DAYE[i]][startJay+_DJAY[i]] = 1;
            visited[startAye+_DAYE[i]][startJay+_DJAY[i]] = true;
            visitQueue.push( pair<short, short>(startAye+_DAYE[i], startJay+_DJAY[i]) );

            dir_max_tile_distance[i] = 1;

            // BFS
            while(!visitQueue.empty()) {
                if (dir_dist_vec[i].size()-1 <= dir_max_tile_distance[i]) {
                    dir_dist_vec[i].push_back(set<unsigned int>());
                }

                log_shortest_path("\tQueue size: %lu\n", visitQueue.size());
                curr = visitQueue.front();
                visitQueue.pop();

                log_shortest_path("\t  @ %d, %d\n", curr.first, curr.second);
                for(int j=0; j<4; j++) {
                    if (isValidPosition(curr.first+_DAYE[j], curr.second+_DJAY[j], false)) {
                        if (!visited[curr.first+_DAYE[j]][curr.second+_DJAY[j]]) {
                            log_shortest_path("\t    %5s unvisited. Pushed into queue.  ", _DIRNAME[j]);
                            pot_map[i][curr.first+_DAYE[j]][curr.second+_DJAY[j]] = pot_map[i][curr.first][curr.second] + 1;
                            visitQueue.push( pair<short, short>(curr.first+_DAYE[j], curr.second+_DJAY[j]) );
                            visited[curr.first+_DAYE[j]][curr.second+_DJAY[j]] = true;

                            if (dir_max_tile_distance[i] < pot_map[i][curr.first][curr.second] + 1) {
                                dir_max_tile_distance[i] = pot_map[i][curr.first][curr.second] + 1;
                            }

                            dir_dist_vec[i][pot_map[i][curr.first][curr.second] + 1].insert(trans_int( curr.first+_DAYE[j], curr.second+_DJAY[j] ));
                        } else {
                            log_shortest_path("\t    %5s visited.\n", _DIRNAME[j]);
                        }
                    } else {
                        log_shortest_path("\t    %5s invalid.\n", _DIRNAME[j]);
                    }
                }
            }

            dir_dist_vec[i].pop_back();
            _dir_max_unvisited_dist[i] = dir_max_tile_distance[i]+1;
            _dir_set_iter[i] = dir_dist_vec[i][_dir_max_unvisited_dist[i]-1].begin();
        } else {
            pot_map[i] = NULL;
            // don't need to change dir_dist_vec
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
    return;
    vector<char> travelOrder[NOprobe];
    PathInfo pathInfo[NOprobe];
    int best_path_idx = 0;
    int best_path_val = -1;
    short curr_aye = startAye;
    short curr_jay = startJay;

    int tmp_explore_count = 0;


    while(cleaned_count < floor_count) {
        int remaining_battery = batteryCapacity;

        /// EXPLORE
        // explore candidate paths
        for(int pi=0; pi < NOprobe; pi++) {
            short aye = curr_aye;
            short jay = curr_jay;
            travelOrder[pi].clear();

            pathInfo[pi].Reset(pi, ++serial_number, remaining_battery);
            log_path_info("\n<path> serial number: %d\n", pathInfo[pi].serial_number);


            // extend outward for half battery capacity
            while(true) {
                //PickDirection(travelOrder[pi], aye, jay, pathInfo[pi]);
                char c = MaxDistOut(travelOrder[pi], aye, jay, pathInfo[pi]);
                if (c == 1) {
                    break;
                } else if (c == 2) {
                    // call ChangeExitDir

                    break;
                }


                else {
                    perror("[ERROR]  Unknown return value of MaxDistPick.\n");
                }
            }
        }

        // for debug.  List all paths.
        best_path_idx = 0;
        best_path_val = -1;
        for(int pi=0; pi < NOprobe; pi++) {
            log_path_info("<path - explore>  #%d    tO steps: %lu    pI steps left: %d\n\t", pi, travelOrder[pi].size(), pathInfo[pi].step_limit);
            for(int ti=0; ti < travelOrder[pi].size(); ti++) {
                log_path_info("%5s ", _DIRNAME[travelOrder[pi][ti]]);
            }
            log_path_info("\n");
        }

        // choose best candidate, and make it official.
                                    /// TODO: if multiple path produces same explore_count, could deploy rating mechanism
        for(int pi=0; pi < NOprobe; pi++) {
            if (pathInfo[pi].explore_count > best_path_val) {
                best_path_idx = pi;
                best_path_val = pathInfo[pi].explore_count;
            }
        }

                                    /// TODO: if even the best path explores 0 new tile, start over.
        tmp_explore_count = 0;
        // for now, break directly
        if (pathInfo[best_path_idx].explore_count == 0) {
            log_path_info("<path>  Even best path cleans 0 new tile.  Break.\n");
            break;
        }
        log_path_info("<path>  Best path(#%d) explored %d new tile.\n", best_path_idx, pathInfo[best_path_idx].explore_count);
        // make it official.  record the best path, update current position.
        
                                                            /////////////////////////////
                                                            /// TODO: remove from dist_vec
                                                            /////////////////////////////


        for(int di = 0; di < travelOrder[best_path_idx].size(); di++) {
            route.push_back(travelOrder[best_path_idx][di]);

            if(travelOrder[best_path_idx][di] == UP) {
                curr_aye --;
            } else if(travelOrder[best_path_idx][di] == DOWN) {
                curr_aye ++;
            } else if(travelOrder[best_path_idx][di] == LEFT) {
                curr_jay --;
            } else if(travelOrder[best_path_idx][di] == RIGHT) {
                curr_jay ++;
            } else {
                printf("SRSLY???  travel toward %d?\n", travelOrder[best_path_idx][di]);
                perror("???");
            }
            log_path_info("\tRegistering %5s", _DIRNAME[travelOrder[best_path_idx][di]]);

            if (!room[curr_aye][curr_jay].visited) {
                log_path_info("\t  The tile hasn't been visited.\n");
                room[curr_aye][curr_jay].visited = true;
                tmp_explore_count ++;
            } else log_path_info("\t  The tile HAS been visited.\n");
        }
        cleaned_count += pathInfo[best_path_idx].explore_count;
        remaining_battery -= pathInfo[best_path_idx].explore_count;


        ///                                             TODO: pathInfo[].step_limit
        PrintMap_Status(curr_aye, curr_jay);

        if (tmp_explore_count != pathInfo[best_path_idx].explore_count) perror("Mismatch explore count.\n");

        /// RETURN
        // returning candidate paths
        for(int pi=0; pi < NOprobe; pi++) {
            short aye = curr_aye;
            short jay = curr_jay;
            travelOrder[pi].clear();

            pathInfo[pi].Reset(pi, ++serial_number, remaining_battery);

            log_path_info("<path> serial number: %d\n", pathInfo[pi].serial_number);

            // extend outward for half battery capacity
            while(true) {
                //PickDirection(travelOrder[pi], aye, jay, pathInfo[pi]);
                char c = Return_Pick(travelOrder[pi], aye, jay, pathInfo[pi]);
                if (c == 1) {
                // taken a step
                    continue;
                } else if (c == 2) {
                // returned to plug
                    break;
                } else {
                    perror("[ERROR]  Unknown return value of Return_pick\n");
                }
            }
        }

        // for debug.  List all paths.
        for(int pi=0; pi < NOprobe; pi++) {
            log_path_info("<path - return>  #%d    tO steps: %lu    pI steps left: %d\n\t", pi, travelOrder[pi].size(), pathInfo[pi].step_limit);
            for(int ti=0; ti < travelOrder[pi].size(); ti++) {
                log_path_info("%5s ", _DIRNAME[travelOrder[pi][ti]]);
            }
            log_path_info("\n");
        }

        best_path_idx = 0;
        best_path_val = -1;
        // choose best retuning path, make it official
        for(int pi = 0; pi < NOprobe; pi++) {
            if (pathInfo[pi].explore_count > best_path_val) {
                best_path_idx = pi;
                best_path_val = pathInfo[pi].explore_count;
            }
        }

        tmp_explore_count = 0;
        for(int di = 0; di < travelOrder[best_path_idx].size(); di++) {
            route.push_back(travelOrder[best_path_idx][di]);

            if(travelOrder[best_path_idx][di] == UP) {
                curr_aye --;
            } else if(travelOrder[best_path_idx][di] == DOWN) {
                curr_aye ++;
            } else if(travelOrder[best_path_idx][di] == LEFT) {
                curr_jay --;
            } else if(travelOrder[best_path_idx][di] == RIGHT) {
                curr_jay ++;
            } else {
                printf("HMMMMMM  travel toward %d?\n", travelOrder[best_path_idx][di]);
            }
            log_path_info("\tRegistering %5s", _DIRNAME[travelOrder[best_path_idx][di]]);

            if (!room[curr_aye][curr_jay].visited) {
                log_path_info("\t  The tile hasn't been visited.\n");
                room[curr_aye][curr_jay].visited = true;
                tmp_explore_count ++;
            } else log_path_info("\t  The tile HAS been visited.\n");
        }
        exit_dir = pathInfo[best_path_idx].exit_dir;
        cleaned_count += pathInfo[best_path_idx].explore_count;


        PrintMap_Status(curr_aye, curr_jay);
        if (tmp_explore_count != pathInfo[best_path_idx].explore_count) perror("Mismatch explore count.\n");
    }
}


char OfflineCoverage::MaxDistOut(vector<char>& travelOrder, short& aye, short& jay, PathInfo& pinfo) {
    set<unsigned int>::iterator tmp;
    char last_exit = -1;
    char enter_dir = -1;
    char c = -1;
    int attempt_count = 0;

    // take the first step
    if (room[aye][jay].isPlug) {
    // starting from plug... of course.
        if (exit_dir > 8) {
        // the very first step.
            log_pick_dir("\tThe very first step, choose random valid direction.\n");
            // for now, choose which ever comes first
            for(int i=0; i<4; i++) {
                if(isValidPosition(aye+_DAYE[i], jay+_DJAY[i], false)) {
                    c = i;
                }
            }
        } else {
        // exit though last entry
            for(int i=0; i<4; i++) {
                if (exit_dir & (1 << i)) {
                    log_pick_dir("\tHave to exit from %s\n", _DIRNAME[i]);
                    c = i;
                }
            }
        }
    }

    last_exit = c;
    travelOrder.push_back(c); 
    aye += _DAYE[c];
    jay += _DJAY[c];
    if (!room[aye][jay].probeTrail[pinfo.id]) room[aye][jay].probeTrail[pinfo.id] = true;
    if (room[aye][jay].serial_number != pinfo.serial_number) room[aye][jay].serial_number = pinfo.serial_number;
    pinfo.step_limit --;


    while(attempt_count < dist_vec[_max_unvisited_dist].size()) {
        // pick a position from dist_vec[max_tile_distance]
        tmp = GetNextPoint();
        attempt_count ++;

        // first check if the shortest path from that position to plug
        //   whether it returns to plug through the same direction as the one we have to exit from
        short tmpAye = getAye_dv(tmp);
        short tmpJay = getJay_dv(tmp);

        // travel to just before plug

                                                        ////////////////////////////////
                                                        /// TODO: could use pot_map! ///
                                                        ////////////////////////////////
        while(room[tmpAye][tmpJay].potential != 1) {
            for(int i=0; i<4; i++) {
                if (isValidPosition(tmpAye+_DAYE[i], tmpJay+_DJAY[i], false)) {
                    if (room[tmpAye+_DAYE[i]][tmpJay+_DJAY[i]].potential <= room[tmpAye][tmpJay].potential) {
                        tmpAye += _DAYE[i];
                        tmpJay += _DJAY[i];
                        break;
                    }
                }
            }
        }

        if (tmpAye < startAye) {
        // UP
            enter_dir = UP;
        } else if (tmpAye > startAye) {
        // DOWN
            enter_dir = DOWN;
        } else {
            if (tmpJay < startJay) {
            // LEFT
                enter_dir = LEFT;
            } else {
            // RIGHT
                enter_dir = RIGHT;
            }
        }


        if (enter_dir != last_exit) {
        // if not, pick next.
            // if every position from dist_vec[max_tile_distance] doesn't return to the tile we have to exit to,
            //   travel directly to the other directions of plug (return a special value, call another function)
            continue;
        } else {
        // if yes, find path from that position to plug, reverse it, becomes a path from plug to that position
            vector<char> revOrder;
            PosToPlug(revOrder, aye, jay, pinfo);

            for (int i=revOrder.size()-1; i>=0; i--) {
                travelOrder.push_back( revOrder[i] );
            }

            return 1; // Path found
        }
    }
    // attempt_count >= dist_vec[_max_unvisited_dist].size()
    return 2; // call ChangeExitDir
}

char OfflineCoverage::Wander(vector<char>& travelOrder, short& aye, short& jay, PathInfo& pinfo) {
    /// TODO.  If there's enough power to wander the vincinity, do this.
}


char OfflineCoverage::ReturnRecharge(vector<char>& travelOrder, short& aye, short& jay, PathInfo& pinfo) {
    // get a path from the current position to plug.
    PosToPlug(travelOrder, aye, jay, pinfo);
    return 1;
}

char OfflineCoverage::ChangeExitDir(vector<char>& travelOrder, short& aye, short& jay, PathInfo& pinfo) {

}

char OfflineCoverage::PosToPlug(vector<char>& travelOrder, short& aye, short& jay, PathInfo& pinfo) {
/// TODO: a path from a position to plug.  with battery limit.

    if (room[aye][jay].potential > pinfo.step_limit) {
        return 0; // do not have enough power to get back.  Which shouldn't be possible!
    }

    while(pinfo.step_limit > 0) {
        if( BFSProbe(travelOrder, aye, jay, pinfo) == 2 ) {
        // arrived at plug.
            char c = *(travelOrder.end()-1);

            if (c == UP) {
                pinfo.exit_dir = 1 << DOWN;
            } else if (c == DOWN) {
                pinfo.exit_dir = 1 << UP;
            } else if (c == LEFT) {
                pinfo.exit_dir = 1 << RIGHT;
            } else if (c == RIGHT) {
                pinfo.exit_dir = 1 << LEFT;
            } else {
                printf("\n\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
                printf("WTF! moved in unknown direction???  c: %d\n", c);
                printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n\n\n");
            }
            return 2; // arrived at plug (as it should)
        } else {
            printf("\n[ERROR]  @ PosToPlug, BFSProbe returned unknown value.\n\n");
        }
    }

    return 1; // failed to return to plug. IT CANT BE HAPPENING!!!!
}

char OfflineCoverage::BFSProbe(vector<char>& travelOrder, short& aye, short& jay, const PathInfo& pinfo) { // don't change step_limit here
// find best local path to return to plug.

    BFS_mini_map mini_map(mini_maxLen, aye, jay);
    vector< vector< BFS_pos > > availPos_vec(mini_maxLen + 1);
    vector< BFS_pos > targetPos_vec;
    bool plug_found = false;

    availPos_vec[0].push_back(BFS_pos(aye, jay));


    for(int li = 1; li <= mini_maxLen; li++) {
        if (li > pinfo.step_limit) {
        // reached limit and still failed to find unvisited tile. random pick
        }

        if (li == mini_maxLen) {
        // only need check those that are furthest away, don't need to add for next time

        } else {
            for(int ti = 0; ti < availPos_vec[li].size(); ti++) {
                for(int i=0; i<4; i++) {
                    // if visited, add adjacent, unchecked tiles that has lower potential to availPos_vec
                    short tmpAye = availPos_vec[li][ti].first + _DAYE[i];
                    short tmpJay = availPos_vec[li][ti].second + _DJAY[i];

                    if (room[tmpAye][tmpJay].isPlug) {
                        if (!plug_found) {
                            targetPos_vec.clear();
                            plug_found = true;
                        }
                        targetPos_vec.push_back(availPos_vec[li][ti]);
                    } else if (!plug_found) {
                        if (room[ tmpAye ][ tmpJay ].visited ||
                              (room[ tmpAye ][ tmpJay ].sequence_number == pinfo.sequence_number && room[ tmpAye ][ tmpJay ].probeTrail[pinfo.id])) {
                        // visited (either officially or in this probe)
                            if(mini_map.isChecked(tmpAye, tmpJay) ) {
                            // checked, don't add
                            } else {
                            // not checked
                                mini_map.Check(tmpAye, tmpJay);
                                if (room[tmpAye][tmpJay].potential > room[availPos_vec[li][ti].first][availPos_vec[li][ti].second].potential) {
                                // has higher potential, dont add
                                } else {
                                // has lower potential, add
                                    availPos_vec[li].push_back( BFS_pos(tmpAye, tmpJay, availPos_vec[li][ti].path, i) );
                                }
                            }
                        } else {
                        // else if unvisited, add to targetPos_vec
                            targetPos_vec.push_back(availPos_vec[li][ti]);
                        }
                    }
                }
            }

            if (targetPos_vec.size() != 0) {
                vector<char> adjacentUnvisited(targetPos_vec.size());
                vector<int> best_indices;
                char lowest = 5;
                for(int tpi = 0; tpi < targetPos_vec.size(); tpi++) {
                    adjacentUnvisited[tpi] = CountAdjacentUnvisited(targetPos_vec[tpi].first, targetPos_vec[tpi].second);
                    if (adjacentUnvisited[tpi] < lowest) lowest = adjacentUnvisited[tpi];
                }

                for(int tpi = 0; tpi < targetPos_vec.size(); tpi++) {
                    if (adjacentUnvisited[tpi] == lowest) best_indices.push_back(tpi);
                }

                // random choose the bests
                int luck_index = best_indices[ GetRand(pinfo.id) % best_indices.size() ];
                BFS_pos& luckyPos = targetPos_vec[luck_index];

                for(int i=0; i < luckPos.path.size(); i++) {

                }
            }

        }
    }
}

inline char OfflineCoverage::CountAdjacentUnvisited(short& aye, short& jay) {
    char tmp = 0;
    for(int i=0; i<4; i++) {
        if (isValidPosition(aye+DAYE[i], jay+_DJAY[i], false)) {
            if (!room[ aye+DAYE[i] ][ jay+_DJAY[i] ].isWall) {
                if (!room[ aye+DAYE[i] ][ jay+_DJAY[i] ].visited) {
                    tmp++;
                }
            }
        }
    }
    return tmp;
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

void OfflineCoverage::Print_dir_dist_vet() {
    _DONOTPRINT;
    for(int i=0; i<4; i++) {
        printf("[DIST_VEC - %s]  max dist: %lu\n", _DIRNAME[i], dir_dist_vec[i].size()-1);
        for(int j=0; j<dir_dist_vec[i].size(); j++) {
            cout << "\t" << j << " steps away:" << "\n";
            for(auto it = dir_dist_vec[i][j].begin(); it!=dir_dist_vec[i][j].end(); it++) {
                cout << "\t  " << getAye_dv(it) << " " << getJay_dv(it) << "\n";
            }
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

/// nope
set<unsigned int>::iterator OfflineCoverage::GetNextPoint(int dir) {
    log_set_op("<set - Get>  GetNextPoint-%s (%d)...\n", _DIRNAME[dir], _dir_max_unvisited_dist[dir]);
    
    if (dir_dist_vec[dir][_dir_max_unvisited_dist[dir]].size() == 0) {
        log_set_op("\tPoints with max distance (%d) all visited.  Lower max.\n", _dir_max_unvisited_dist[dir]);
        
        while(dir_dist_vec[dir][--_dir_max_unvisited_dist[dir]].size() == 0) {
            log_set_op("\t  dir_dist_vec[%d][%d] is empty as well.  Lower!, dir.\n", dir, _dir_max_unvisited_dist[dir]);
        }

        log_set_op("\t  Done.  dir_dist_vec[%d][%d] has %lu entry.\n", dir, _dir_max_unvisited_dist[dir], dir_dist_vec[dir][_dir_max_unvisited_dist[dir]].size());
        _dir_set_iter[dir] = dir_dist_vec[dir][_dir_max_unvisited_dist[dir]].begin();
    }

    if (_dir_set_iter[dir] == dir_dist_vec[dir][_dir_max_unvisited_dist[dir]].end()) {
        log_set_op("\tReached end for max distance(%d)\n", _dir_max_unvisited_dist[dir]);
        _dir_set_iter[dir] = dir_dist_vec[dir][_dir_max_unvisited_dist[dir]].begin();
    }
    
    return _dir_set_iter[dir]++;
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

/// nope
void OfflineCoverage::RemovePoint(short aye, short jay, int potential, int dir) {
    log_set_op("<set - Remove>  Removing %d, %d from dir_dist_vec[%d][%d]\n", aye, jay, dir, potential);
    set<unsigned int>::iterator iter = dir_dist_vec[dir][potential].find(trans_int(aye, jay));
    if (iter == dir_dist_vec[dir][potential].cend()) {
        perror("\tNot found in dist_vec!\n");
    } else {
        _set_iter = dir_dist_vec[dir][potential].erase(iter);
        printf("\t  dir_dist_vec[%d][%d].size(): %lu\n", dir, potential, dir_dist_vec[dir][potential].size());
        if (_set_iter != dir_dist_vec[dir][potential].cend()) {
            //cout << "..." << getAye_dv(_set_iter) << " " << getJay_dv(_set_iter) << "\n";
        }
        else {
            cout << "it's the end...\n";
        }
    }
}

void OfflineCoverage::TestSet() {
    return;
    for(int i=0; i<4; i++) {
        if (dir_dist_vec[i].size() == 0) {
            break;
        }
        printf("dir_dist_vec for %s:\n", _DIRNAME[i]);
        for(int n=0; n<floor_count; n++) {
            set<unsigned int>::iterator val = GetNextPoint(i);
            printf("(pot: %d) %d, %d\n", pot_map[i][getAye_dv(val)][getJay_dv(val)], getAye_dv(val), getJay_dv(val));
            RemovePoint(getAye_dv(val), getJay_dv(val), pot_map[i][getAye_dv(val)][getJay_dv(val)], i);
        }
    }
}