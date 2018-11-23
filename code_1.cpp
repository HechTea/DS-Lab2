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

    PathInfo () {}
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
    int probe_id;
    bool probeTrail[NOprobe];

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
    int floor_count, cleaned_count;
    int probe_id;

    unsigned char randarr[NOprobe];
    unsigned char mask[NOprobe];

    char PickDirection(vector<char>& travelOrder, short& posAye, short& posJay, PathInfo& pinfo);
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
    oc.PrintMap_Dist();
    oc.PrintMap_Equipotential();
    oc.CalcDist();
    return 0;
}



OfflineCoverage::OfflineCoverage(int height, int width, int batteryCapacity) :
    height(height), width(width), batteryCapacity(batteryCapacity)
{
    extend_length = batteryCapacity / 2;
    floor_count = cleaned_count = 0;

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

/*
todo:
    how many tiles are half battery life away
*/
void OfflineCoverage::CalcShortestPath() { // calculate potential
    queue< pair<short, short> > visitQueue;
    pair<short, short> curr;

    room[startAye][startJay].potential = 0;
    visitQueue.push( {startAye, startJay} );

    // BFS
    while(!visitQueue.empty()) {
        curr = visitQueue.front();
        visitQueue.pop();


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


    for(int pi=0; pi < NOprobe; pi++) {
        short aye = startAye;
        short jay = startJay;
        pathInfo[pi].id = pi;

        // extend outward
        for(int bi=0; bi < extend_length; bi++) {
            PickDirection(travelOrder[pi], aye, jay, pathInfo[pi]);
        }
    }
    for(int pi=0; pi < NOprobe; pi++) {
        log_path_info("[Path]  #%d", pi);
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

void OfflineCoverage::PrintMap_Specials() {

}




