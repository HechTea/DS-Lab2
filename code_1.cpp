
#include <stdio.h>
#include <string.h>
#include <fstream>
#include <iostream>
#include <queue>
#include <vector>
#include <math.h>
#include <random>

#define _DONOTPRINT //return
#define NOprobe 1
//#define EXPLR_BFS_MAXLEN 4
#define log_read_info //printf
#define log_shortest_path //printf
#define log_pick_dir //printf
#define log_path_info printf
#define log_output //printf
#define log_progress printf

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

    void PrintMap_Raw();
    void PrintMap_Dist();
    void PrintMap_Dist_adj();
    void PrintMap_Equipotential();
    void PrintMap_Status(short curr_aye, short curr_jay);
    void PrintMap_Status_Path(int id);
    void PrintMap_Specials();

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

    int EXPLR_BFS_MAXLEN = 50500; // absolute maximum distance in 1000x1000 map

    int height, width, batteryCapacity;
    Tile ** room;
    int ** pot_map[4];
    vector<char> route;


    short startAye, startJay;
    int extend_length;
    int floor_count = 0, cleaned_count = 0;
    char exit_dir = 1+2+4+8;
    int probe_id;
    unsigned short serial_number;
    int maxTiles = 0;

    unsigned char randarr[NOprobe];
    unsigned char mask[NOprobe];

    char PickDirection(vector<char>& travelOrder, short& posAye, short& posJay, PathInfo& pinfo);
    char BFSPick(vector<char>& travelOrder, short& aye, short& jay, PathInfo& pinfo);
    char Return_Pick(vector<char>& travelOrder, short& aye, short& jay, PathInfo& pinfo);
    inline char isValidPosition(short aye, short jay, bool allowWall);
    inline int RateTile_out(short aye, short jay, int pi);
    inline void RateBase(short aye, short jay, int& rating);
};



int main(int argc, char** argv) {
    int height, width, batteryCapacity;

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
    //oc.CalcShortestPath_adj();
    oc.PrintMap_Dist();
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

    // BFS
    while(!visitQueue.empty()) {
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
                } else {
                    log_shortest_path("\t    %5s visited.\n", _DIRNAME[i]);
                }
            } else {
                log_shortest_path("\t    %5s invalid.\n", _DIRNAME[i]);
            }
        }
    }
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

void OfflineCoverage::CalcDist() { // the main function


    vector<char> travelOrder[NOprobe];
    PathInfo pathInfo[NOprobe];
    int best_path_idx = 0;
    int best_path_val = -1;
    short curr_aye = startAye;
    short curr_jay = startJay;

    int tmp_explore_count = 0;

    int n = 0;
    int t = 0;

    while(cleaned_count < floor_count) {
        if(n % 1 == 0) {
            printf("%5d\n", t++);
        }
        n++;

        int remaining_battery = batteryCapacity;

        /// EXPLORE
        // explore candidate paths
        for(int pi=0; pi < NOprobe; pi++) {
            short aye = curr_aye;
            short jay = curr_jay;
            travelOrder[pi].clear();

            pathInfo[pi].Reset(pi, ++serial_number, remaining_battery);
            log_path_info("<path> serial number: %d\n", pathInfo[pi].serial_number);


                        /// ////////////////////////////////////////////
                        /// Instead of only half battery, give it full control,
                        ///   Just make sure whichever tile it went, it can return to recharge
                        /// ////////////////////////////////////////////
            // extend outward for half battery capacity
            while(true) {
                //PickDirection(travelOrder[pi], aye, jay, pathInfo[pi]);
                char c = BFSPick(travelOrder[pi], aye, jay, pathInfo[pi]);
                if (c == 0) {
                // adjacent tile is unvisited, but didn't choose any because battery would run out
                    break;
                } else if (c == 1) {
                // advanced 1 step
                    continue;
                } else if (c == 2) {
                // no unvisited tile within reach
                    ///                 TODO: 2 actions.  return to plug in shortest path,
                    ///                   or wander further till half bat to see if it can get luck on the way to recharge
                    break;
                } else if (c == 3) {
                // multiple steps taken.
                    continue;
                } else if (c == 4) {
                // hit a corner
                    ///                 TODO: Either wander, or return.  Could be combined with c==2
                    break;
                } else {
                    perror("[ERROR]  Unknown return value of BFSPick.\n");
                }
            }
        }

        ///                                     TODO: if there's still power left, could add it to the returning stage

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

char OfflineCoverage::PickDirection(vector<char>& travelOrder, short& posAye, short& posJay, PathInfo& pinfo) {
    int rating[4] = {0};
    int highestRating = -300;
    int highestDir = -1;
    /*
    mask = mask << 1;
    if (mask == 0) mask = 1;
    */
    log_pick_dir("@ %d, %d\n", posAye, posJay);

    // check if probeTrail needs update
    if (room[posAye][posJay].probe_id != probe_id) {
        memset(room[posAye][posJay].probeTrail, 0, sizeof(bool)*NOprobe);
        room[posAye][posJay].probe_id = probe_id;
    }

    for(int dir = 0; dir < 4; dir++) { // conveniently u,d,l,r
        if (!isValidPosition(posAye+_DAYE[dir], posJay+_DJAY[dir], false)) {
            continue;
        } else {
            if (room[posAye+_DAYE[dir]][posJay+_DJAY[dir]].visited ||
                room[posAye+_DAYE[dir]][posJay+_DJAY[dir]].probeTrail[pinfo.id]) { // visited
                rating[dir] -= 200;
            }
            rating[dir] += RateTile_out(posAye+_DAYE[dir], posJay+_DJAY[dir], pinfo.id);
        }
    }


    // choose the best direction (random select if ties)
    for(int dir=0; dir<4; dir++) {
        log_pick_dir("\t%5s rates %d\n", _DIRNAME[dir], rating[dir]);
        if(rating[dir] > highestRating) {
            highestRating = rating[dir];
            highestDir = dir;
        } else if(rating[dir] == highestRating) {
            mask[pinfo.id] = mask[pinfo.id] << 1;
            if (mask[pinfo.id] == 0) mask[pinfo.id] = 1;

            if(randarr[pinfo.id] & mask[pinfo.id]) {
                highestRating = rating[dir];
                highestDir = dir;
            }
        }
    }
    log_pick_dir("\t  choose %s\n", _DIRNAME[highestDir]);
    // update current pos for this path
    travelOrder.push_back(highestDir);
    posAye += _DAYE[highestDir];
    posJay += _DJAY[highestDir];
    if (!room[posAye][posJay].visited &&
        !room[posAye][posJay].probeTrail[pinfo.id]) { // not visited
        pinfo.explore_count ++;
    }
    if (highestDir == -1) {
        log_pick_dir("EMWTF\n");
        return 0;
    }
    return 1;
}

char OfflineCoverage::BFSPick(vector<char>& travelOrder, short& aye, short& jay, PathInfo& pinfo) {
    //PrintMap_Status_Path(pinfo.id);

    vector<char> availVec;
    char c = -1;



    log_pick_dir("<Exploring> @ %d, %d    battery: %d\n", aye, jay, pinfo.step_limit);

    if (room[aye][jay].isPlug) {
    // we're starting from plug, only 1 direction to choose from (unless it's the first step)
        log_pick_dir("\tOh, starting from plug!\n");
        if (exit_dir > 8) {
        // yup, first step
            log_pick_dir("\t  But it's the very first step, so any direction is available.\n");
        } else {
            for(int i=0; i<4; i++) {
                if (exit_dir & (1 << i)) {
                    log_pick_dir("\t  And we have to exit from %s\n", _DIRNAME[i]);

                    travelOrder.push_back(i);
                    aye += _DAYE[i];
                    jay += _DJAY[i];
                    if (!room[aye][jay].probeTrail[pinfo.id]) room[aye][jay].probeTrail[pinfo.id] = true;
                    if (room[aye][jay].serial_number != pinfo.serial_number) room[aye][jay].serial_number = pinfo.serial_number;
                    pinfo.step_limit --;
                    return 1;
                }
            }
        }
    }




    // find nearest tile that hasn't been visited
    //  record those that meet the criteria



    for(int i=0; i<4; i++) {
        if (isValidPosition(aye+_DAYE[i], jay+_DJAY[i], false)) {
            // check if visited
            if (room[aye+_DAYE[i]][jay+_DJAY[i]].visited) {
            // if this direction is conformed to have been visited
                log_pick_dir("\t  %5s has been visited\n", _DIRNAME[i]);
                continue;
            } else {
            // else check if this path has been visited for this particular probe
                if (room[aye+_DAYE[i]][jay+_DJAY[i]].serial_number != pinfo.serial_number) {
                // if the tile.probe_trail records -not- the status for this probe
                //   then it must haven't been visited, since this tile isn't conformed to have been visited
                //   then it is clear.
                    log_pick_dir("\t    SrNo. of %d, %d was %d, updated to %d\n", aye+_DAYE[i], jay+_DJAY[i], room[aye+_DAYE[i]][jay+_DJAY[i]].serial_number, pinfo.serial_number);
                    room[aye+_DAYE[i]][jay+_DJAY[i]].serial_number = pinfo.serial_number;
                    room[aye+_DAYE[i]][jay+_DJAY[i]].probeTrail[pinfo.id] = false;
                } else {
                // else, tile.probe_trail -does- record the status for this probe,
                // meaning the info is valid for use, then check if it's been visited
                    log_pick_dir("\t    SrNo. of %d, %d up to date, ", aye+_DAYE[i], jay+_DJAY[i]);
                    if (room[aye+_DAYE[i]][jay+_DJAY[i]].probeTrail[pinfo.id]) {
                    // been visited, continue looking
                        log_pick_dir("but has been visited.\n");
                        continue;
                    } // else, it is clear.
                    log_pick_dir("and hasn't been visited.\n");
                }
            }

            if (room[aye+_DAYE[i]][jay+_DJAY[i]].potential >= room[aye][jay].potential) {
                availVec.push_back(i);
            } else {
                log_pick_dir("\t    But moves closer to plug.\n");
            }
        }
    }


    if (availVec.size() != 0) {
        if (availVec.size() == 1) {
        // only one adjacent tile haven't been visited
            log_pick_dir("\tOnly 1 way to choose\n");
            c = availVec.front();
            if (room[aye+_DAYE[c]][jay+_DJAY[c]].potential > pinfo.step_limit) {
                log_pick_dir("\t  Which is %5s, which is too far from plug.\n", _DIRNAME[c]);
                return 0; // didn't choose any.
            }
        } else {
        // there's more than 1 adjacent tile that haven't been visited
            log_pick_dir("\tThere are %lu ways to choose: ", availVec.size());
            for(int i=0; i<availVec.size(); i++) {
                log_pick_dir("%5s ", _DIRNAME[availVec[i]]);
            }
            log_pick_dir("\n");
            // if using thread, spread out; if not, random pick
            //      Note: for thread, need to limit max thread count, once reached, still need to random pick

            // for now, just random pick

            vector<char> target;
            for(int i=0; i<availVec.size(); i++) {
                if (room[aye+_DAYE[availVec[i]]][jay+_DJAY[availVec[i]]].potential <= pinfo.step_limit) {
                    target.push_back(availVec[i]);
                }
            }

            if (target.size() == 0) {
                log_pick_dir("\t  But all of them are too far from plug.\n");
                return 0; // didn't choose any.
            } else {
                c = target[GetRand(pinfo.id) % target.size()];
//                log_pick_dir("\t  And %d of which is valid: ", target.size());
//                for(int i=0; i<target.size(); i++) {
//                    log_pick_dir("%5s ", _DIRNAME[target[i]]);
//                }
//                log_pick_dir("\n");
            }
        }

        travelOrder.push_back(c);
        aye += _DAYE[c];
        jay += _DJAY[c];
        pinfo.step_limit --;

        if (!room[aye][jay].visited) {
        // if that way hasn't been visited
            if (!room[aye][jay].probeTrail[pinfo.id]) {
            // and if it is also unvisited in this probe
                pinfo.explore_count ++;
                room[aye][jay].probeTrail[pinfo.id] = true;
            }
        }
        log_pick_dir("\n");
    } else {
    // all 4 adjacent tiles have been visited
        vector< vector< BFS_pos > > availPos_vec(EXPLR_BFS_MAXLEN);
        vector< BFS_pos > targetPos_vec;
        // Note: availDir_vec[ '0' ] actually means available directions '1' step away

        // for now, use this stupid method.
        bool checked[height][width];
        memset(checked, false, sizeof(bool)*height*width);

        log_pick_dir("\tAll adjacent tiles have been visited\n");

        // first, add valid positions fulfilling the following criteria to availPos_vec
        //   - hasn't been officially visited
        //   - hasn't been visited in this probe
        //   - moves away from plug
                                        /// TODO: need tweak here.
        checked[aye][jay] = true;
        for(int i = 0; i < 4; i++) {
            if (isValidPosition(aye+_DAYE[i], jay+_DJAY[i], false)) {
                availPos_vec[0].push_back( BFS_pos(aye+_DAYE[i], jay+_DJAY[i], i) );
                checked[aye+_DAYE[i]][jay+_DJAY[i]] = true;
//                if (!room[aye+_DAYE[i]][jay+_DJAY[i]].visited) {
//                // The direction hasn't been visited
//                    if (room[aye+_DAYE[i]][jay+_DJAY[i]].serial_number != pinfo.serial_number) {
//                    // Hasn't been visited in this probe either
//                        // update serial number of this tile
//                        room[aye+_DAYE[i]][jay+_DJAY[i]].serial_number = pinfo.serial_number;
//
//                        if(room[aye+_DAYE[i]][jay+_DJAY[i]].potential >= room[aye][jay].potential) {
//                        // only check those that leads further from plug
//                            // doesn't need to check the "checked[][]" cuz... yeah.
//                        }
//                    }
//                }
            }
        }

        // then, BFS until EXPLR_BFS_MAXLEN or found an unvisited tile
        //  note: once found, need to check if it's further than pinfo.step_limit

                                        ///   ALSO!  even though tiles that are EXPLR_BFS_MAXLEN steps away are added into availPos_vec
                                        ///      they weren't checked.
        for(int li = 1; li < EXPLR_BFS_MAXLEN; li++) {
            if (li+1 > pinfo.step_limit) {
            // too far away.  even if found an unvisited tile, the robot wouldn't be able to get back to plug
                log_pick_dir("\t  Cannot find any unvisited tile near enough to get to and return for recharge. (%d)\n", pinfo.step_limit);
                return 2; // meaning ^^^^^
            }
            /// Now that step_limit is initially assigned as battery_capacity, this ^^^^^ shouldn't be possible

            log_pick_dir("\t  Checking tiles %d steps away.\n", li);
            if (availPos_vec[li-1].size() == 0) {
                log_pick_dir("\t    No tiles to check.\n");
                break;
            } else {
                log_pick_dir("\t    %lu tiles to check.\n", availPos_vec[li-1].size());
            }

            for(int di = 0; di < availPos_vec[li-1].size(); di++) {
                log_pick_dir("\t    @ %d %d\n", availPos_vec[li-1][di].first, availPos_vec[li-1][di].second);

                // else, the direction HAS been visited
                //   add valid tiles adjacent to the direction to availPos_vec
                log_pick_dir("\t\tHAS been visited. recored valid adjacent tiles:\n\t\t  ");
                for(int i=0; i<4; i++) {
                    log_pick_dir("\t\t%5s @ %d %d\n", _DIRNAME[i], availPos_vec[li-1][di].first+_DAYE[i], availPos_vec[li-1][di].second+_DJAY[i]);

                    if (isValidPosition(availPos_vec[li-1][di].first+_DAYE[i], availPos_vec[li-1][di].second+_DJAY[i], false)) {
                        if (!room[availPos_vec[li-1][di].first+_DAYE[i]][availPos_vec[li-1][di].second+_DJAY[i]].visited) {
                        // if the direction is not conformed to have been visited
                            if (room[availPos_vec[li-1][di].first+_DAYE[i]][availPos_vec[li-1][di].second+_DJAY[i]].serial_number != pinfo.serial_number) {
                            // and the direction truly is unvisited, score!
                                // then figure out how to get there...
                                log_pick_dir("\t\t  SrNo. of %d, %d was %d, updated to %d (meaning that this truly hasn't been visited)\n",
                                             availPos_vec[li-1][di].first+_DAYE[i], availPos_vec[li-1][di].second+_DJAY[i],
                                             room[availPos_vec[li-1][di].first+_DAYE[i]][availPos_vec[li-1][di].second+_DJAY[i]].serial_number,
                                             pinfo.serial_number);
                                room[availPos_vec[li-1][di].first+_DAYE[i]][availPos_vec[li-1][di].second+_DJAY[i]].serial_number = pinfo.serial_number;
                                room[availPos_vec[li-1][di].first+_DAYE[i]][availPos_vec[li-1][di].second+_DJAY[i]].probeTrail[pinfo.id] = false;

                                // ~~~ teleport AAA ~~~

                            } else {
                            // else, tile.probe_trail -does- record the status for this probe,
                            // meaning the info is valid for use, then check if it's been visited
                                if (!room[availPos_vec[li-1][di].first+_DAYE[i]][availPos_vec[li-1][di].second+_DJAY[i]].probeTrail[pinfo.id]) {
                                    log_pick_dir("\t\t  SrNo. of %d, %d up to date, and hasn't been visited\n", availPos_vec[li-1][di].first+_DAYE[i], availPos_vec[li-1][di].second+_DJAY[i]);

                                    // ~~~ teleport AAA ~~~

                                } else {
                                // truly HAS been visited
                                    log_pick_dir("\t\t  SrNo. of %d, %d up to date, and HAS been visited (in this probe at least)\n", availPos_vec[li-1][di].first+_DAYE[i], availPos_vec[li-1][di].second+_DJAY[i]);
                                    continue;
                                }
                            }

                            // ~~~ AAA ~~~
                            // only the directions that are unvisited get to this part.
                            if (room[availPos_vec[li-1][di].first+_DAYE[i]][availPos_vec[li-1][di].second+_DJAY[i]].potential
                                >= room[availPos_vec[li-1][di].first][availPos_vec[li-1][di].second].potential) {
                            // only check those that leads further from plug
                                /// hasn't been visited, goes away from plug.  add into targetPos_vec
                                log_pick_dir("\t\t  This has higher potential. BINGO!\n");
                                targetPos_vec.push_back(availPos_vec[li-1][di]);
                                continue;
                            } else {
                            // leads closer to plug. don't add
                                log_pick_dir("\t\t  This moves close to plug. Don't add\n");
                                continue;
                            }
                        } else {
                        // this direction has been visited, decide if adding its adjacent tiles
                            log_pick_dir("\t\t  Has been officially visited.  Decide whether to add into queue...\n");
                            if (targetPos_vec.size() == 0) {
                            // only add the neighbor of this direction into BFS queue if no unvisited tiles had been found
                                log_pick_dir("\t\t    No unvisited tile has been found so far...");
                                if (checked[availPos_vec[li-1][di].first+_DAYE[i]][availPos_vec[li-1][di].second+_DJAY[i]]) {
                                    // this position has been added to BFS queue, or something...
                                    log_pick_dir("  but no, cuz it has already been added\n");
                                    continue;
                                } else {
                                    log_pick_dir("  and yup.  New to the queue.  Adding.\n");
                                    availPos_vec[li].push_back( BFS_pos(availPos_vec[li-1][di].first+_DAYE[i],
                                                                        availPos_vec[li-1][di].second+_DJAY[i],
                                                                        availPos_vec[li-1][di].path, i) );
                                    checked[availPos_vec[li-1][di].first+_DAYE[i]][availPos_vec[li-1][di].second+_DJAY[i]] = true;
                                }
                            }
                        }
                    }
                }
            }


            if (li == EXPLR_BFS_MAXLEN-1 && targetPos_vec.size() == 0) {
                // check tiles at max distance
                log_pick_dir("\t  Checking tiles %d (max) steps away.\n", li);
                if (availPos_vec[li].size() == 0) {
                    log_pick_dir("\t    No tiles to check.\n");
                    break;
                } else {
                    log_pick_dir("\t    %lu tiles to check.\n", availPos_vec[li].size());
                }

                for (int di = 0; di < availPos_vec[li].size(); di++) {
                    log_pick_dir("\t\t@ %d %d ... ", availPos_vec[li][di].first, availPos_vec[li][di].second);
                    if (!room[availPos_vec[li][di].first][availPos_vec[li][di].second].visited) {
                        if (!room[availPos_vec[li][di].first][availPos_vec[li][di].second].probeTrail[pinfo.id]) {
                            targetPos_vec.push_back(availPos_vec[li-1][di]);
                        } else {
                            log_pick_dir("has been visited in this probe.\n");
                        }
                    } else {
                        log_pick_dir("is officially visited.\n");
                    }
                }
            }


            if (targetPos_vec.size() != 0) {
                log_pick_dir("\t    There are %lu tiles %d steps away that are unvisited\n", targetPos_vec.size(), li);

                BFS_pos& tmp = targetPos_vec[GetRand(pinfo.id) % targetPos_vec.size()];
                log_pick_dir("\t    Chose %d, %d\n", tmp.first, tmp.second);
                log_pick_dir("\t\thow to get here: ");


                for(int i = 0; i < tmp.path.size(); i++) {
                    c = tmp.path[i];
                    travelOrder.push_back(c);
                    aye += _DAYE[c];
                    jay += _DJAY[c];
                    pinfo.step_limit --;

                    if (!room[aye][jay].visited) {
                    // if that way hasn't been visited
                        if (!room[aye][jay].probeTrail[pinfo.id]) {
                        // and if it is also unvisited in this probe
                            pinfo.explore_count ++;
                            room[aye][jay].probeTrail[pinfo.id] = true;
                        }
                    }

                    log_pick_dir("%s ", _DIRNAME[c]);
                }
                log_pick_dir("\n");
                return 3; // means that multiple steps have been taken
            }
        }
        if (c == -1) {
            log_pick_dir("\t  No tile within %d steps is unvisited\n", EXPLR_BFS_MAXLEN);
                            /// pick any direction, or the furtherest in availPos_vec
            // for now pick the direction that moves away from plug
            log_pick_dir("\t  Check which direction moves away from plug. (pot: %d)\n", room[aye][jay].potential);
            // availVec is currently empty
            for(int i = 0; i < 4; i++) {
                c = i;
                if (isValidPosition(aye+_DAYE[c], jay+_DJAY[c], false)) {
                    log_pick_dir("\t    %5s (pot: %d)... ", _DIRNAME[c], room[aye+_DAYE[c]][jay+_DJAY[c]].potential);
                    if (room[aye+_DAYE[c]][jay+_DJAY[c]].potential > room[aye][jay].potential) {
                        log_pick_dir("good.\n");
                        availVec.push_back(c);
                        continue;
                    }
                } else {
                    log_pick_dir("\t    %5s invalid.\n", _DIRNAME[c]);
                }
            }

            if (availVec.size() == 0) {
            // in a corner...
                return 4;
            }
            // random pick
            c = availVec[GetRand(pinfo.id) % availVec.size()];
            travelOrder.push_back(c);
            aye += _DAYE[c];
            jay += _DJAY[c];
            pinfo.step_limit --;

            if (!room[aye][jay].visited) {
            // if that way hasn't been visited
                if (!room[aye][jay].probeTrail[pinfo.id]) {
                // and if it is also unvisited in this probe
                    pinfo.explore_count ++;
                    room[aye][jay].probeTrail[pinfo.id] = true;
                }
            }
        }
    }



    log_pick_dir("\t  Choose %s\n", _DIRNAME[c]);
    return 1;
}

char OfflineCoverage::Return_Pick(vector<char>& travelOrder, short& aye, short& jay, PathInfo& pinfo) {
    // idea: use Tile.potential to determine how to move toward to plug
    log_pick_dir("<Returning> @ %d, %d    pot: %d\n", aye, jay, room[aye][jay].potential);
    vector<char> visitQueue;
    char c;
    ///                                     TODO: probeTrail, serial_number
    ///                                             (currently wouldn't cross the same tile twice, so hadn't implement it)
    // first, record candidates to choose from.
    //   Current filter: tile is floor or plug, is closer to plug than (aye,jay)
    for(int i=0; i<4; i++) {
        // check if a direction is available
        if (isValidPosition(aye+_DAYE[i], jay+_DJAY[i], true)) { // plug is also a wall, allow walls for now, filter later
            // if the tile is a wall but not a plug
            if(room[aye+_DAYE[i]][jay+_DJAY[i]].isWall && !room[aye+_DAYE[i]][jay+_DJAY[i]].isPlug) continue;

            // else check if the direction brings it closer to the plug
            if (room[aye+_DAYE[i]][jay+_DJAY[i]].potential <= room[aye][jay].potential) {
                                // Note: it 'seems' to be impossible for adjacent tiles to have the same potential,
                                //     so both < and <= probably works
                visitQueue.push_back(i);
            } ///                           TODO: if battery allows, could choose to go further from plug
        }
    }

    if (visitQueue.size() == 1) {
    // only 1 meets criteria, choose that one
        log_pick_dir("\tOnly 1 way to choose\n");
        c = visitQueue.front();
        travelOrder.push_back(c);
        aye += _DAYE[c];
        jay += _DJAY[c];
        pinfo.step_limit --;

        if (!room[aye][jay].visited) {
        // if that way hasn't been visited
            if (!room[aye][jay].probeTrail[pinfo.id]) {
            // and if it is also unvisited in this probe
                pinfo.explore_count ++;
                room[aye][jay].probeTrail[pinfo.id] = true;
            }
        }
    } else if (visitQueue.size() != 0) {
    // more than 1 to choose from
        log_pick_dir("\tThere are %lu ways to choose\n", visitQueue.size());
        log_pick_dir("\t  ");
        for(int i = 0; i<visitQueue.size(); i++) {
            log_pick_dir("%5s ", _DIRNAME[visitQueue[i]]);
        }
        log_pick_dir("\n");
        // could spread out if using thread, could do random pick
        //  for now, prioritize unvisited direction, pick the first unvisited tile found in the order udlr
                                                                /// TODO here ^^^^^
        int unvisited_idx = -1;
        for(int i = 0; i<visitQueue.size(); i++) {
            if(room[aye+_DAYE[visitQueue[i]]][jay+_DJAY[visitQueue[i]]].visited) {
            // if this direction is conformed to have been visited
                log_pick_dir("\t  %5s has been visited\n", _DIRNAME[visitQueue[i]]);
                continue;
            } else {
            // else check if this path has been visited for this particular probe
                if (room[aye+_DAYE[visitQueue[i]]][jay+_DJAY[visitQueue[i]]].serial_number != pinfo.serial_number) {
                // if the tile.probe_trail records -not- the status for this probe
                //   then it must haven't been visited, since this tile isn't conformed to have been visited
                //   then it is clear.
                    log_pick_dir("\t    SrNo. of %d, %d was %d, updated to %d\n", aye+_DAYE[visitQueue[i]], jay+_DJAY[visitQueue[i]], room[aye+_DAYE[visitQueue[i]]][jay+_DJAY[visitQueue[i]]].serial_number, pinfo.serial_number);
                    room[aye+_DAYE[visitQueue[i]]][jay+_DJAY[visitQueue[i]]].serial_number = pinfo.serial_number;
                } else {
                // else, tile.probe_trail -does- record the status for this probe,
                // meaning the info is valid for use, then check if it's been visited
                    log_pick_dir("\t    SrNo. of %d, %d up to date, ", aye+_DAYE[visitQueue[i]], jay+_DJAY[visitQueue[i]]);
                    if (room[aye+_DAYE[visitQueue[i]]][jay+_DJAY[visitQueue[i]]].probeTrail[pinfo.id]) {
                    // been visited, continue looking
                        log_pick_dir("but has been visited.\n");
                        continue;
                    } // else, it is clear.
                    log_pick_dir("and hasn't been visited.\n");
                }
            }
            unvisited_idx = i;
        }
        if (unvisited_idx == -1) {
        // all directions that has lower potential have been visited
        // for now, random pick
        ///                                         TODO: BFS here
            c = visitQueue[GetRand(pinfo.id) % visitQueue.size()];
            travelOrder.push_back(c);
            aye += _DAYE[c];
            jay += _DJAY[c];
            pinfo.step_limit --;
            if (!room[aye][jay].visited) {
            // if that way hasn't been visited
                if (!room[aye][jay].probeTrail[pinfo.id]) {
                // and if it is also unvisited in this probe
                    pinfo.explore_count ++;
                    room[aye][jay].probeTrail[pinfo.id] = true;
                }
            }
        } else {
            c = visitQueue[unvisited_idx];
            travelOrder.push_back(c);
            aye += _DAYE[c];
            jay += _DJAY[c];
            pinfo.step_limit --;
            if (!room[aye][jay].visited) {
            // if that way hasn't been visited
                if (!room[aye][jay].probeTrail[pinfo.id]) {
                // and if it is also unvisited in this probe
                    pinfo.explore_count ++;
                    room[aye][jay].probeTrail[pinfo.id] = true;
                }
            }

        }
    } else {
    /// all 4 tiles doesn't meet criteria, which currently only limits to tiles that bring the robot closer to plug
    ///     but there will always be at least one.
        perror("\tERROR!\n");
    }

    log_pick_dir("\t  Chose %s\n", _DIRNAME[c]);



    if (room[aye][jay].isPlug) {
    // if we've arrived at plug
        log_pick_dir("\t  And arrived at plug!\n");
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

        return 2;
    }
    return 1;
}

inline char OfflineCoverage::isValidPosition(short aye, short jay, bool allowWall) {
    return (aye >= 0 && jay >= 0 && aye <= height-1 && jay <= width-1 && !(!allowWall && room[aye][jay].isWall));
}

inline int OfflineCoverage::RateTile_out(short aye, short jay, int pi) {
    int rating = room[aye][jay].baseRating;
    for(short daye = -1; daye <= 1; daye++) {
        for(short djay = -1; djay <= 1; djay ++) {
            if (daye == 0 && djay == 0) {
                continue;
            } else if (!isValidPosition(aye + daye, jay + djay, true)) {
                continue;
            } else {
                if (room[aye+daye][jay+djay].visited ||
                    room[aye+daye][jay+djay].probeTrail[pi]) { // visited
                    if(daye == 0 || djay == 0) { // not corner tile
                        rating += 20;
                    } else {
                        rating += 5;
                    }
                } else { // not visited
                    if (room[aye+daye][jay+djay].isWall) { // wall will never be visited
                        rating += 50;
                    } else {
                        rating += 70;
                    }
                }
            }
        }
    }
    return rating;
}

void OfflineCoverage::RateBase(short aye, short jay, int& rating) {
    rating = 0;
    if (isValidPosition(aye, jay, false)) {
        for(short daye = -1; daye <= 1; daye++) {
            for(short djay = -1; djay <= 1; djay++) {
                if (daye == 0 && djay == 0) { // self must be a floor tile
                    continue;
                } else if (!isValidPosition(aye+daye, jay+djay, true)) {
                    continue;
                } else {
                    if (room[aye+daye][jay+djay].isWall) {
                        rating += 50;
                    } else {
                        rating += 20;
                    }
                }
            }
        }
    }
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

void OfflineCoverage::PrintMap_Status_Path(int id) {
    _DONOTPRINT;
    printf("\n[MAP STATUS]\n\tHeight: %d, Width: %d, Battery: %d\n", height, width, batteryCapacity);

    for(short aye=0; aye<height; aye++) {
        for(short jay=0; jay<width; jay++) {
            if (room[aye][jay].isPlug) {
                printf("%c ", _PLUG);
            } else if (room[aye][jay].isWall) {
                printf("%c ", _WALL);
            } else {
                if (room[aye][jay].probeTrail[id]) {
                    printf("  ");
                } else {
                    printf("%c ", _FLOOR);
                }
            }
        }
        printf("\n");
    }
}

void OfflineCoverage::PrintMap_Specials() {

}




