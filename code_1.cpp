#include <stdio.h>
#include <string.h>
#include <queue>
#include <vector>
#include <math.h>
#include <random>

#define NOprobe 1
#define log_read_info //printf
#define log_pick_dir printf
#define log_path_info printf

using namespace std;

enum {UP = 0, DOWN = 1, LEFT = 2, RIGHT = 3};
enum {UPi = 0, DOWNi = 1, LEFTi = 2, RIGHTi = 3};

short _DAYE[4] = {-1, 1, 0, 0};
short _DJAY[4] = {0, 0, -1, 1};
char _DIRNAME[4][6] = {"UP", "DOWN", "LEFT", "RIGHT"};

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

    PathInfo () {}
    void Reset() {
        explore_count = 0;
        rating = 0;
        id = -1;
        serial_number = 0;
        exit_dir = 0;
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
    void CalcDist();

    void PrintMap_Raw();
    void PrintMap_Dist();
    void PrintMap_Equipotential();
    void PrintMap_Status();
    void PrintMap_Status_Path(int id);
    void PrintMap_Specials();


private:
    const unsigned int INF = (unsigned int)-1;
    const char _WALL = 'W';
    const char _FLOOR = '.';
    const char _PLUG = 'R';
    const char _SADDLE = '^';
    const char _SPLIT = '*';

    int height, width, batteryCapacity;
    Tile ** room;


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



int main(void) {
    int height, width, batteryCapacity;

    scanf("%d %d %d\n", &height, &width, &batteryCapacity);

    OfflineCoverage oc(height, width, batteryCapacity);
    oc.ReadMap();
    oc.PrintMap_Raw();
    oc.CalcShortestPath(); // calculate potential
    //oc.PrintMap_Dist();
    //oc.PrintMap_Equipotential();
    oc.CalcDist();
    return 0;
}



OfflineCoverage::OfflineCoverage(int height, int width, int batteryCapacity) :
    height(height), width(width), batteryCapacity(batteryCapacity)
{
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
        for(short jay=0; jay<width; jay++) {
            c = getchar(); // 1/0
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
                startAye = aye;
                startJay = jay;
            }
            else {
                log_read_info("    WTF.\n");
            }

            getchar(); // space or new line
        }
    }

    for(short aye=0; aye<height; aye++) {
        for(short jay=0; jay<width; jay++) {
            RateBase(aye, jay, room[aye][jay].baseRating);
        }
    }
}

void OfflineCoverage::CalcShortestPath() { // calculate potential
    queue< pair<short, short> > visitQueue;
    pair<short, short> curr;

    room[startAye][startJay].potential = 0;
    visitQueue.push( {startAye, startJay} );

    // BFS
    while(!visitQueue.empty()) {
        curr = visitQueue.front();
        visitQueue.pop();
        // count tiles at max distance
        if (room[curr.first][curr.second].potential >= batteryCapacity / 2) {
            maxTiles ++;
        }

        // left
        if (curr.second > 0) {
            if (!room[curr.first][curr.second-1].isWall) { // isn't wall
                if (room[curr.first][curr.second-1].potential == INF) { // hasn't been visited
                    room[curr.first][curr.second-1].potential = room[curr.first][curr.second].potential + 1;
                    visitQueue.push( {curr.first, curr.second-1} );
                }
            }
        }
        // right
        if (curr.first < width - 1) {
            if (!room[curr.first][curr.second+1].isWall) { // isn't wall
                if (room[curr.first][curr.second+1].potential == INF) { // hasn't been visited
                    room[curr.first][curr.second+1].potential = room[curr.first][curr.second].potential + 1;
                    visitQueue.push( {curr.first, curr.second+1} );
                }
            }
        }
        // up
        if (curr.first > 0) {
            if (!room[curr.first-1][curr.second].isWall) { // isn't wall
                if (room[curr.first-1][curr.second].potential == INF) { // hasn't been visited
                    room[curr.first-1][curr.second].potential = room[curr.first][curr.second].potential + 1;
                    visitQueue.push( {curr.first-1, curr.second} );
                }
            }
        }
        // down
        if (curr.first < height - 1) {
            if (!room[curr.first+1][curr.second].isWall) { // isn't wall
                if (room[curr.first+1][curr.second].potential == INF) { // hasn't been visited
                    room[curr.first+1][curr.second].potential = room[curr.first][curr.second].potential + 1;
                    visitQueue.push( {curr.first+1, curr.second} );
                }
            }
        }
    }
}

void OfflineCoverage::CalcDist() { // the main function


    vector<char> travelOrder[NOprobe];
    PathInfo pathInfo[NOprobe];
    int best_path_idx = 0;
    int best_path_val = -1;
    short curr_aye = startAye;
    short curr_jay = startJay;

    while(cleaned_count < floor_count) {
        /// EXPLORE
        // explore candidate paths
        for(int pi=0; pi < NOprobe; pi++) {
            short aye = curr_aye;
            short jay = curr_jay;
            travelOrder[pi].clear();
            pathInfo[pi].Reset();
            pathInfo[pi].id = pi;
            pathInfo[pi].explore_count = 0;
            serial_number++;
            pathInfo[pi].serial_number = serial_number;

            // extend outward for half battery capacity
            for(int bi=0; bi < extend_length; bi++) {
                //PickDirection(travelOrder[pi], aye, jay, pathInfo[pi]);
                BFSPick(travelOrder[pi], aye, jay, pathInfo[pi]);
            }
        }

        for(int pi=0; pi < NOprobe; pi++) {
            log_path_info("[Path]  #%d\n\t", pi);
            for(int ti=0; ti < travelOrder[pi].size(); ti++) {
                log_path_info("%5s ", _DIRNAME[travelOrder[pi][ti]]);
            }
        } log_path_info("\n");

        // choose best candidate, and make it official.
                                    /// TODO: if multiple path produces same explore_count, could deploy rating mechanism
        for(int pi=0; pi < NOprobe; pi++) {
            if(pathInfo[pi].explore_count > best_path_val) {
                best_path_idx = pi;
                best_path_val = pathInfo[pi].explore_count;
            }
        }
                                    /// TODO: if even the best path explores 0 new tile, start over.
        for(int di = 0; di < travelOrder[best_path_idx].size(); di++) {
            if(travelOrder[best_path_idx][di] == UP) {
                log_path_info("\tRegistering %s", _DIRNAME[travelOrder[best_path_idx][di]]);
                curr_aye --;
            } else if(travelOrder[best_path_idx][di] == DOWN) {
                log_path_info("\tRegistering %s", _DIRNAME[travelOrder[best_path_idx][di]]);
                curr_aye ++;
            } else if(travelOrder[best_path_idx][di] == LEFT) {
                log_path_info("\tRegistering %s", _DIRNAME[travelOrder[best_path_idx][di]]);
                curr_jay --;
            } else if(travelOrder[best_path_idx][di] == RIGHT) {
                log_path_info("\tRegistering %s", _DIRNAME[travelOrder[best_path_idx][di]]);
                curr_jay ++;
            } else {
                printf("SRSLY???  travel toward %d?\n", travelOrder[best_path_idx][di]);
            }
            if (!room[curr_aye][curr_jay].visited) {
                log_path_info("\t  The tile hasn't been visited.\n");
                room[curr_aye][curr_jay].visited = true;
            } else log_path_info("\t  The tile HAS been visited.\n");
        }
        cleaned_count += pathInfo[best_path_idx].explore_count;
        PrintMap_Status();

        /// RETURN
        // returning candidate paths
        for(int pi=0; pi < NOprobe; pi++) {
            short aye = curr_aye;
            short jay = curr_jay;
            travelOrder[pi].clear();
            pathInfo[pi].Reset();
            pathInfo[pi].id = pi;
            pathInfo[pi].explore_count = 0;
            serial_number++;
            pathInfo[pi].serial_number = serial_number;

            // extend outward for half battery capacity
            for(int bi=0; bi < batteryCapacity - extend_length; bi++) {
                //PickDirection(travelOrder[pi], aye, jay, pathInfo[pi]);
                if (Return_Pick(travelOrder[pi], aye, jay, pathInfo[pi]) == 2) {
                // returned to plug
                    break;
                }
            }
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

        for(int di = 0; di < travelOrder[best_path_idx].size(); di++) {
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
            if (!room[curr_aye][curr_jay].visited) room[curr_aye][curr_jay].visited = true;
        }
        exit_dir = pathInfo[best_path_idx].exit_dir;
        cleaned_count += pathInfo[best_path_idx].explore_count;
        PrintMap_Status();
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
    char c;
    log_pick_dir("<Exploring> @ %d, %d\n", aye, jay);

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
                    return 1;
                }
            }
        }
    }

    // left barrel shift randarr
    randarr[pinfo.id] = (randarr[pinfo.id] << 1) + ((randarr[pinfo.id] & (1 << 8)) ? 1 : 0);


    // find nearest tile that hasn't been visited
    //  record those that meet the criteria
    for(int i=0; i<4; i++) {
        if (isValidPosition(aye+_DAYE[i], jay+_DJAY[i], false)) {
            // check if visited
            if (room[aye+_DAYE[i]][jay+_DJAY[i]].visited) {
            // if this direction is conformed to have been visited
                continue;
            } else {
            // else check if this path has been visited for this particular probe
                if (room[aye+_DAYE[i]][jay+_DJAY[i]].serial_number != pinfo.serial_number) {
                // if the tile.probe_trail records -not- the status for this probe
                //   then it must haven't been visited, since this tile isn't conformed to have been visited
                //   then it is clear.
                    log_pick_dir("\t    SrNo. of %d, %d was %d, updated to %d\n", aye+_DAYE[i], jay+_DJAY[i], room[aye+_DAYE[i]][jay+_DJAY[i]].serial_number, pinfo.serial_number);
                    room[aye+_DAYE[i]][jay+_DJAY[i]].serial_number = pinfo.serial_number;
                } else {
                // else, tile.probe_trail -does- record the status for this probe,
                // meaning the info is valid for use, then check if it's been visited
                    if (room[aye+_DAYE[i]][jay+_DJAY[i]].probeTrail[pinfo.id]) {
                    // been visited, continue.
                        continue;
                    } // else, it is clear.
                }
            }

            availVec.push_back(i);
        }
    }

    if (availVec.size() == 1) {
    // only one adjacent tile haven't been visited
        log_pick_dir("\tOnly 1 way to choose\n");
        c = availVec.front();
    } else if (availVec.size() != 0) {
    // there's more than 1 adjacent tile that haven't been visited
        log_pick_dir("\tThere are %d ways to choose\n", availVec.size());
        // if using thread, spread out; if not, random pick
        //      Note: for thread, need to limit max thread count, once reached, still need to random pick

        // for now, just random pick
        c = availVec[randarr[pinfo.id] % availVec.size()];
    } else {
    // all 4 adjacent tiles have been visited
        log_pick_dir("\tAll adjacent tiles have been visited.\n");
        // BFS the nearest unvisited tile, need to set max dist for BFS. once reached, random pick

        ///          TODO: BFS
        //              Note: might not have to check tiles that brings the robot closer to plug
        // for now, choose the first valid tile in the order uldr
        for(int i=0; i<4; i++) {
            if(isValidPosition(aye+_DAYE[i], jay+_DJAY[i], false)) {
                c = i;
            }
        }
    }

    travelOrder.push_back(c);
    aye += _DAYE[c];
    jay += _DJAY[c];
    if (!room[aye][jay].probeTrail[pinfo.id]) {
        pinfo.explore_count ++;
        room[aye][jay].probeTrail[pinfo.id] = true;
    }

    log_pick_dir("\t  Choose %s\n", _DIRNAME[c]);
    return 1;
}

char OfflineCoverage::Return_Pick(vector<char>& travelOrder, short& aye, short& jay, PathInfo& pinfo) {
    // idea: use Tile.potential to determine how to move toward to plug
    log_pick_dir("<Returning> @ %d, %d\n", aye, jay);
    vector<char> visitQueue;
    char c;
    ///                                     TODO: probeTrail
    // first, record candidates to choose from.
    //   Current filter: tile is floor or plug, is closer to plug than (aye,jay)
    for(int i=0; i<4; i++) {
        // check if a direction is available
        if (isValidPosition(aye+_DAYE[i], jay+_DJAY[i], true)) { // plug is also a wall, allow walls for now, filter later
            // if the tile is a wall but not a plug
            if(room[aye+_DAYE[i]][jay+_DJAY[i]].isWall && !room[aye+_DAYE[i]][jay+_DJAY[i]].isPlug) continue;

            // then check if it brings it closer to the plug
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

    } else if (visitQueue.size() != 0) {
    // more than 1 to choose from
        log_pick_dir("\tThere are %d ways to choose\n", visitQueue.size());
        // could spread out if using thread, could do random pick (maybe prioritize unvisited tiles)

        // for now just random pick
        c = visitQueue[randarr[pinfo.id] % visitQueue.size()];
    } else {
    /// all 4 tiles doesn't meet criteria, which currently only limits to tiles that bring the robot closer to plug
    ///     which there will always be at least one.
        printf("\tERROR!\n");
    }

    log_pick_dir("\t  Chose %s\n", _DIRNAME[c]);

    travelOrder.push_back(c);
    aye += _DAYE[c];
    jay += _DJAY[c];
    if (!room[aye][jay].probeTrail[pinfo.id]) {
        pinfo.explore_count ++;
        room[aye][jay].probeTrail[pinfo.id] = true;
    }


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
    return (aye > 0 && jay > 0 && aye+1 < height && jay+1 < width && !(!allowWall && room[aye][jay].isWall));
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

void OfflineCoverage::PrintMap_Equipotential() {
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

void OfflineCoverage::PrintMap_Status() {
    printf("\n[MAP STATUS]\n\tFloor count %d  Cleaned count %d\n",floor_count, cleaned_count);

    for(short aye=0; aye<height; aye++) {
        for(short jay=0; jay<width; jay++) {
            if (room[aye][jay].isPlug) {
                printf("%c ", _PLUG);
            } else if (room[aye][jay].isWall) {
                printf("%c ", _WALL);
            } else {
                if (room[aye][jay].visited) {
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




