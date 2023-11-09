//
//  Witness.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 10/20/18.
//  Copyright © 2018 University of Denver. All rights reserved.
//
#ifndef HOG2_ENVIRONMENTS_WITNESS_H
#define HOG2_ENVIRONMENTS_WITNESS_H

#include <algorithm>
#include <array>
#include <bitset>
#include <cassert>
#include <cstring>
#include <iostream>
#include <numeric>
#include <regex>
#include <sstream>
#include <string>

#include "SearchEnvironment.h"
#include "vectorCache.h"

template<int width, int height>
int GetEdgeHash(bool horizontal, int x, int y)
{
    return (horizontal) ? y * (width) + x : width * (height + 1) + y * (width + 1) + x;
}

template<int width, int height>
int GetEdgeHash(int x1, int y1, int x2, int y2)
{
    if (y1 == y2)
        return GetEdgeHash<width, height>(true, std::min(x1, x2), y1);
    else if (x1 == x2)
        return GetEdgeHash<width, height>(false, x1, std::min(y1, y2));
    assert(false);
}

template<int width, int height>
class WitnessState {
public:
    std::vector<std::pair<int, int>> path;
    std::bitset<(width + 1) * (height + 1)> occupiedCorners;
    std::bitset<(width + 1) * (height) + (width) * (height + 1)> occupiedEdges;

    WitnessState() { Reset(); }

    WitnessState(const WitnessState<width, height> &state)
    {
        path = state.path;
        occupiedCorners = state.occupiedCorners;
        occupiedEdges = state.occupiedEdges;
    }

    void Reset()
    {
        path.resize(0);
        occupiedCorners.reset();
        occupiedEdges.reset();
    }

    bool Occupied(int which)
    {
        if (which < width * (height + 1))
        {
            int x = which % (width);
            int y = which / (width);
            return OccupiedEdge(x, y, x + 1, y);
        }
        which -= width * (height + 1);
        if (which < (width + 1) * height)
        {
            int x = which % (width + 1);
            int y = which / (width + 1);
            return OccupiedEdge(x, y, x, y + 1);
        }
        which -= (width + 1) * height;
        // constraint on corner
        int x = which % (width + 1);
        int y = which / (width + 1);
        return Occupied(x, y);
        // AddCannotCrossConstraint(x, y);
    }

    bool IsAlongTheWall() const
    {
        if (path.size() < 2)
            return false;
        auto &prevPos = path[path.size() - 2];
        auto &currPos = path.back();
        return ((currPos.first == 0 || currPos.first == width) && currPos.first == prevPos.first) ||
            ((currPos.second == 0 || currPos.second == height) && currPos.second == prevPos.second);
    }

    bool HitTheWall() const
    {
        if (path.size() <= 1)
            return false;
        auto &p = path.back();
        return (p.first == 0 || p.first == width) ||
            (p.second == 0 || p.second == height);
    }

    bool Occupied(int x, int y) const { return occupiedCorners[y * (width + 1) + x]; }

    void Occupy(int x, int y) { occupiedCorners.set(y * (width + 1) + x, true); }

    void Unoccupy(int x, int y)
    {
        if (x >= 0 && x <= width && y >= 0 && y <= height) occupiedCorners.set(y * (width + 1) + x, false);
    }

    bool OccupiedEdge(int x1, int y1, int x2, int y2) const
    {
        return occupiedEdges[GetEdgeHash<width, height>(x1, y1, x2, y2)];
    }

    void OccupyEdge(int x1, int y1, int x2, int y2) { occupiedEdges.set(GetEdgeHash<width, height>(x1, y1, x2, y2)); }

    void UnoccupyEdge(int x1, int y1, int x2, int y2)
    {
        occupiedEdges.set(GetEdgeHash<width, height>(x1, y1, x2, y2), false);
    }

    bool InGoal() const
    {
        return path.back().first < 0 || path.back().first > width ||
            path.back().second < 0 || path.back().second > height;
    }

    bool ShareThePath(const WitnessState<width, height> &state)
    {
        for (size_t i = 0; i < (path.size() < state.path.size() ? path.size() : state.path.size()); ++i)
        {
            if (path[i] != state.path[i])
                return false;
        }
        return true;
    }
};

enum WitnessAction {
    kUp,
    kDown,
    kLeft,
    kRight,
    kStart,
    kEnd
};

template<int width, int height>
static bool operator==(const WitnessState<width, height> &a, const WitnessState<width, height> &b)
{
    return a == b;
}

template<int width, int height>
class InteractiveWitnessState {
public:
    void Reset()
    {
        ws.Reset();
        frac = 0;
        currState = kWaitingStart;
    }

    void IncrementTime()
    {
        if (currState != kWaitingStart) return;
        frac += 0.04;
        if (frac > 3) frac = 0;
    }

    WitnessState<width, height> ws;
    // for drawing
    float frac;
    std::pair<int, int> target; // where we are heading next
    WitnessAction targetAct;

    enum controlState {
        kWaitingStart,
        kInPoint,
        kBetweenPoints,
        kWaitingRestart
    };
    controlState currState;
};

enum WitnessRegionConstraintType {
    kNoRegionConstraint = 0,
    kSeparation = 1,
    kStar = 2,
    kTetris = 3,
    kNegativeTetris = 4,
    kTriangle = 5,
    kEraser = 6,
    kRegionConstraintCount = 7
};

/**
 * @see https://github.com/thefifthmatt/windmill-client/blob/master/src/grid.proto#L45
 */
static inline std::ostream& operator<<(std::ostream &os, const rgbColor &color)
{
    os << std::quoted("color") << ": ";
    if (color == Colors::black)
        os << "1";
    else if (color == Colors::white)
        os << "2";
    else if (color == Colors::cyan)
        os << "3";
    else if (color == Colors::magenta)
        os << "4";
    else if (color == Colors::yellow)
        os << "5";
    else if (color == Colors::red)
        os << "6";
    else if (color == Colors::green)
        os << "7";
    else if (color == Colors::blue)
        os << "8";
    else if (color == Colors::orange)
        os << "9";
    else
        os << "0";
    return os;
}

static inline const rgbColor& GetColorFromEnum(int t)
{
    switch (t) {
        case 1:
            return Colors::black;
        case 2:
            return Colors::white;
        case 3:
            return Colors::cyan;
        case 4:
            return Colors::magenta;
        case 5:
            return Colors::yellow;
        case 6:
            return Colors::red;
        case 7:
            return Colors::green;
        case 8:
            return Colors::blue;
        default:
            return Colors::orange;
    }
}

static inline int GetTetrisParameterFromString(int width, const std::string &grid)
{
    switch (width) {
        case 1:
        {
            if (grid == "[true]")
                return 1;
            if (grid == "[true,true]")
                return 3;
            if (grid == "[true,true,true]")
                return 9;
            if (grid == "[true,true,true,true]")
                return 20;
            return 0;
        }
        case 2:
        {
            if (grid == "[true,true]")
                return 2;
            if (grid == "[true,true,true,false]")
                return 4;
            if (grid == "[true,true,false,true]")
                return 5;
            if (grid == "[true,false,true,true]")
                return 6;
            if (grid == "[false,true,true,true]")
                return 7;
            if (grid == "[true,true,true,true]")
                return 10;
            if (grid == "[true,true,true,false,true,false]")
                return 15;
            if (grid == "[true,true,false,true,false,true]")
                return 16;
            if (grid == "[true,false,true,false,true,true]")
                return 17;
            if (grid == "[false,true,false,true,true,true]")
                return 18;
            if (grid == "[true,false,true,true,true,false]")
                return 23;
            if (grid == "[true,false,true,true,true,false]")
                return 24;
            return 0;
        }
        case 3:
        {
            if (grid == "[true,true,true]")
                return 8;
            if (grid == "[true,true,true,true,false,false]")
                return 11;
            if (grid == "[true,false,false,true,true,true]")
                return 12;
            if (grid == "[true,true,true,false,false,true]")
                return 13;
            if (grid == "[true,true,true,true,false,false]")
                return 14;
            if (grid == "[true,true,true,false,true,false]")
                return 21;
            if (grid == "[false,true,false,true,true,true]")
                return 22;
            return 0;
        }
        case 4:
        {
            if (grid == "[true,true,true,true]")
                return 19;
            return 0;
        }
        default:
            return 0;
    }
}

struct WitnessRegionConstraint {
    WitnessRegionConstraintType type;
    int parameter{};
    rgbColor color;

    bool operator==(const WitnessRegionConstraint &a) const
    {
        return a.type == this->type && a.parameter == this->parameter && a.color == this->color;
    }
    bool operator!=(const WitnessRegionConstraint &a) const
    {
        return !(*this == a);
    }

    /**
     * @see https://github.com/thefifthmatt/windmill-client/blob/master/src/grid.proto#L3
     */
    explicit operator std::string() const
    {
        std::stringstream ss;
        ss << "{";
        switch (type) {
            case kSeparation:
            {
                ss << std::quoted("type") << ": 7, " << color;
                break;
            }
            case kStar:
            {
                ss << std::quoted("type") << ": 8, " << color;
                break;
            }
            case kTetris:
            case kNegativeTetris:
            {
                ss << std::quoted("type") << ": 9, "
                    << std::quoted("shape") << ": {"
                    << std::quoted("width") << ": ";
                switch (parameter)
                {
                    case 1:
                        ss << 1 << ", " << std::quoted("grid") << ": [true]";
                        break;
                    case 2:
                        ss << 2 << ", " << std::quoted("grid") << ": [true, true]";
                        break;
                    case 3:
                        ss << 1 << ", " << std::quoted("grid") << ": [true, true]";
                        break;
                    case 4:
                        ss << 2 << ", " << std::quoted("grid") << ": [true, true, true, false]";
                        break;
                    case 5:
                        ss << 2 << ", " << std::quoted("grid") << ": [true, true, false, true]";
                        break;
                    case 6:
                        ss << 2 << ", " << std::quoted("grid") << ": [true, false, true, true]";
                        break;
                    case 7:
                        ss << 2 << ", " << std::quoted("grid") << ": [false, true, true, true]";
                        break;
                    case 8:
                        ss << 3 << ", " << std::quoted("grid") << ": [true, true, true]";
                        break;
                    case 9:
                        ss << 1 << ", " << std::quoted("grid") << ": [true, true, true]";
                        break;
                    case 10:
                        ss << 2 << ", " << std::quoted("grid") << ": [true, true, true, true]";
                        break;
                    case 11:
                        ss << 3 << ", " << std::quoted("grid") << ": [true, true, true, true, false, false]";
                        break;
                    case 12:
                        ss << 3 << ", " << std::quoted("grid") << ": [true, false, false, true, true, true]";
                        break;
                    case 13:
                        ss << 3 << ", " << std::quoted("grid") << ": [true, true, true, false, false, true]";
                        break;
                    case 14:
                        ss << 3 << ", " << std::quoted("grid") << ": [true, true, true, true, false, false]";
                        break;
                    case 15:
                        ss << 2 << "," << std::quoted("grid") << ": [true, true, true, false, true, false]";
                        break;
                    case 16:
                        ss << 2 << "," << std::quoted("grid") << ": [true, true, false, true, false, true]";
                        break;
                    case 17:
                        ss << 2 << "," << std::quoted("grid") << ": [true, false, true, false, true, true]";
                        break;
                    case 18:
                        ss << 2 << "," << std::quoted("grid") << ": [false, true, false, true, true, true]";
                        break;
                    case 19:
                        ss << 4 << ", " << std::quoted("grid") << ": [true, true, true, true]";
                        break;
                    case 20:
                        ss << 1 << ", " << std::quoted("grid") << ": [true, true, true, true]";
                        break;
                    case 21:
                        ss << 3 << ", " << std::quoted("grid") << ": [true, true, true, false, true, false]";
                        break;
                    case 22:
                        ss << 3 << ", " << std::quoted("grid") << ": [false, true, false, true, true, true]";
                        break;
                    case 23:
                        ss << 2 << ", " << std::quoted("grid") << ": [true, false, true, true, true, false]";
                        break;
                    case 24:
                        ss << 2 << ", " << std::quoted("grid") << ": [true, false, true, true, true, false]";
                        break;
                    default:
                        ss << 0;
                        break;
                }
                if (type == kNegativeTetris)
                    ss << std::quoted("negate") << ": true";
                ss << "}";
                break;
            }
            case kTriangle:
            {
                ss << std::quoted("type") << ": 11, "
                    << std::quoted("triangle_count") << ": " << parameter;
                break;
            }
            default:
                break;
        }
        ss << "}";
        return ss.str();
    }
};

enum WitnessPathConstraintType {
    kNoPathConstraint = 0,
    kMustCross = 1,
    kCannotCross = 2,
    kPathConstraintCount = 3
};

template<int width, int height>
class Witness : public SearchEnvironment<WitnessState<width, height>, WitnessAction> {
public:
    enum legality {
        kLegal,
        kNotAtEnd,
        kNotAtStart,
        kNotValidAction,
        kHitCannotCross,
        kHitStart,
        kHitLine
    };

    // Must cross edge/node
    struct mustCrossEdgeConstraint {
        bool horiz;
        std::pair<int, int> location;

        bool operator==(const mustCrossEdgeConstraint &a)
        {
            return a.horiz == this->horiz && a.location == this->location;
        }
    };

    /* Hexagon constraints - path must cross this point */
    static constexpr int GetNumPathConstraints();

    std::array<WitnessPathConstraintType, Witness<width, height>::GetNumPathConstraints()> pathConstraints;
    std::array<std::pair<Graphics::point, Graphics::rect>,
        Witness<width, height>::GetNumPathConstraints()> pathConstraintLocations;

    //	std::vector<mustCrossEdgeConstraint> mustCrossEdgeConstraints;
    //	std::vector<std::pair<int, int>> mustCrossConstraints;
    //
    //	std::vector<mustCrossEdgeConstraint> cannotCrossEdgeConstraints;
    //	std::vector<std::pair<int, int>> cannotCrossConstraints;

    struct separationObject {
        separationObject() : valid(false), color(Colors::pink) {}

        bool valid;
        rgbColor color;
    };

    std::array<std::array<WitnessRegionConstraint, height>, width> regionConstraints;
    //	constraint constraints[width][height];
    std::array<int, (int)kRegionConstraintCount> constraintCount;

    // TODO: merge these
    //	std::vector<separationObject> separationConstraints;
    //	int separationCount;
    //
    //	std::vector<int> tetrisConstraints;
    //	int tetrisCount;
    //
    //	std::vector<int> triangleConstraints;
    //	int triangleCount;
    std::array<std::pair<Graphics::point, Graphics::rect>, width*height> regionConstraintLocations;

    Witness() // :separationConstraints(width*height), separationCount(0), tetrisConstraints(width*height),
    // tetrisCount(0)
    {
        static_assert(
                (width <= 8) && (height <= 8) && (width > 0) && (height > 0),
                "Error: width/height must be between 1...8");
        Reset();
    }

    Witness(const Witness<width, height> &w)
    {
        *this = w;
        //		pathConstraints = w.pathConstraints;
        ////		mustCrossEdgeConstraints = w.mustCrossEdgeConstraints;
        ////		mustCrossConstraints = w.mustCrossConstraints;
        ////		cannotCrossEdgeConstraints = w.cannotCrossEdgeConstraints;
        ////		cannotCrossConstraints = w.cannotCrossConstraints;
        //
        //		constraints = w.constraints;
        //		constraintCount = w.constraintCount;
        //		start = w.start;
        //		goal = w.goal;
    }

    Witness<width, height> &operator=(const Witness<width, height> &w)
    {
        pathConstraints = w.pathConstraints;
        //		mustCrossEdgeConstraints = w.mustCrossEdgeConstraints;
        //		mustCrossConstraints = w.mustCrossConstraints;
        //		cannotCrossEdgeConstraints = w.cannotCrossEdgeConstraints;
        //		cannotCrossConstraints = w.cannotCrossConstraints;
        regionConstraints = w.regionConstraints;
        constraintCount = w.constraintCount;
        start = w.start;
        goal = w.goal;
        goalMap = w.goalMap;
        return *this;
    }

    void Reset()
    {
        for (int c = 0; c < kRegionConstraintCount; c++)
            constraintCount[c] = 0;
        constraintCount[kNoRegionConstraint] = width * height;

        for (int x = 0; x < width; x++)
        {
            for (int y = 0; y < height; y++)
            {
                regionConstraints[x][y].type = kNoRegionConstraint;
                Graphics::point p1 = GetScreenCoord(x, y);
                Graphics::point p2 = GetScreenCoord(x + 1, y + 1);
                Graphics::point p3 = (p1 + p2) * 0.5;
                regionConstraintLocations[x + y * width] = std::make_pair(p3, Graphics::rect{p3, 0.15});
            }
        }

        for (int x = 0; x < GetNumPathConstraints(); x++)
            pathConstraints[x] = kNoPathConstraint;

        for (int x = 0; x < width; x++)
        {
            for (int y = 0; y <= height; y++)
            {
                Graphics::point p = (GetScreenCoord(x, y) + GetScreenCoord(x + 1, y)) * 0.5;
                // horizontal
                pathConstraintLocations[x + y * width] = std::make_pair(p, Graphics::rect{p, 0.05});
            }
        }
        for (int x = 0; x <= width; x++)
        {
            for (int y = 0; y < height; y++)
            {
                Graphics::point p = (GetScreenCoord(x, y) + GetScreenCoord(x, y + 1)) * 0.5;
                // vertical
                pathConstraintLocations[width * (height + 1) + x * height + y] =
                        std::make_pair(p, Graphics::rect{p, 0.05});
            }
        }
        for (int x = 0; x < width + 1; x++)
        {
            for (int y = 0; y < height + 1; y++)
            {
                Graphics::point p = GetScreenCoord(x, y);
                // vertex
                pathConstraintLocations[width * (height + 1) + (width + 1) * height + (width + 1) * y + x] =
                        std::make_pair(p, Graphics::rect{p, 0.05});
            }
        }

        //		mustCrossConstraints.clear();
        //		mustCrossEdgeConstraints.clear();
        //		cannotCrossConstraints.clear();
        //		cannotCrossEdgeConstraints.clear();
        start.clear();
        start.emplace_back(0, 0);
        SetGoal(width, height + 1);
        //		goal.clear();
        //		goal.push_back({width,height});
        //		goal.push_back({width-1,height});
        //		goal.push_back({width,height-1});
    }

    void GetSuccessors(
            const WitnessState<width, height> &nodeID, std::vector <WitnessState<width, height>> &neighbors) const;

    void GetActions(const WitnessState<width, height> &nodeID, std::vector <WitnessAction> &actions) const;

    void ApplyAction(WitnessState<width, height> &s, WitnessAction a) const;

    void ApplyAction(std::pair<int, int> &s, WitnessAction a) const;

    bool InvertAction(WitnessAction &a) const;

    void UndoAction(WitnessState<width, height> &s, WitnessAction a) const;

    bool Legal(WitnessState<width, height> &s, WitnessAction a) const;

    bool Legal(WitnessState<width, height> &s, WitnessAction a, legality &l) const;

    /** Heuristic value between two arbitrary nodes. **/
    double HCost(const WitnessState<width, height> &node1, const WitnessState<width, height> &node2) const;

    double GCost(const WitnessState<width, height> &node1, const WitnessState<width, height> &node2) const;

    double GCost(const WitnessState<width, height> &node, const WitnessAction &act) const;

    bool GoalTest(const WitnessState<width, height> &node, const WitnessState<width, height> &goal) const;

    bool GoalTest(const WitnessState<width, height> &node) const;

    bool RegionTest(const WitnessState<width, height> &node) const;

    uint64_t GetMaxHash() const;

    uint64_t GetStateHash(const WitnessState<width, height> &node) const;

    void GetStateFromHash(uint64_t parent, WitnessState<width, height> &s) const;

    uint64_t GetActionHash(WitnessAction act) const;

    void OpenGLDraw() const {};

    void OpenGLDraw(const WitnessState<width, height> &) const {};

    /** Draw the transition at some percentage 0...1 between two states */
    void OpenGLDraw(const WitnessState<width, height> &, const WitnessState<width, height> &, float) const {};

    void OpenGLDraw(const WitnessState<width, height> &, const WitnessAction &) const {};

    void GLLabelState(const WitnessState<width, height> &, const char *) const {}; // draw label over state
    void GLDrawLine(const WitnessState<width, height> &x, const WitnessState<width, height> &y) const {};

    void DrawRegionConstraint(
            Graphics::Display &display, const WitnessRegionConstraint &constraint, const Graphics::point &p3) const;

    void Draw(Graphics::Display &display) const;

    void Draw(Graphics::Display &display, const WitnessState<width, height> &) const;

    void Draw(Graphics::Display &display, const InteractiveWitnessState<width, height> &) const;

    // returns true if the goal was reached
    bool Click(Graphics::point, InteractiveWitnessState<width, height> &ws);

    void Move(Graphics::point, InteractiveWitnessState<width, height> &ws);

    const WitnessRegionConstraint& GetRegionConstraint(int x, int y) const
    {
        return regionConstraints[x][y];
    }

    void SetStart(int x, int y)
    {
        start.clear();
        start.emplace_back(x, y);
    }

    void AddStart(int x, int y) { start.emplace_back(x, y); }

    void SetGoal(int x, int y)
    {
        goal.clear();
        std::fill(goalMap.begin(), goalMap.end(), 0);
        AddGoal(x, y);
    }

    bool AddGoal(int x, int y)
    {
        if (x == -1 || x == width + 1 || y == -1 || y == height + 1)
        {
            goal.emplace_back(x, y);
            if (x > width) x = width;
            if (x < 0) x = 0;
            if (y > height) y = height;
            if (y < 0) y = 0;
            // this is the location from which we reach that goal (note: off by 1 to keep semantics of 0)
            goalMap[GetPathIndex(x, y)] = (int)goal.size();
            return true;
        }
        else
        {
            printf("Error: invalid goal location\n");
            return false;
        }
    }

    void ClearPathConstraints() { pathConstraints = {kNoPathConstraint}; }

    //{ mustCrossConstraints.clear(); mustCrossEdgeConstraints.clear(); }
    void SetMustCrossConstraint(int);

    bool GetMustCrossConstraint(int) const;

    bool GetMustCrossConstraint(int, int) const;

    bool GetMustCrossConstraint(bool, int, int) const;

    void ClearMustCrossConstraint(int);

    void AddMustCrossConstraint(bool horiz, int x, int y);    // { mustCrossEdgeConstraints.push_back({horiz, {x, y}});}
    void AddMustCrossConstraint(int x, int y);                // { mustCrossConstraints.push_back({x, y});}
    void AddMustCrossConstraint(int);                         // { mustCrossConstraints.push_back({x, y});}
    void RemoveMustCrossConstraint(bool horiz, int x, int y); // { mustCrossEdgeConstraints.pop_back();}
    void RemoveMustCrossConstraint(int x, int y);             // { mustCrossConstraints.pop_back();}

    /* Hexagon constraints - cannot cross this point */
    constexpr int GetNumCannotCrossConstraints() const;

    void SetCannotCrossConstraint(int);

    bool GetCannotCrossConstraint(int) const;

    bool GetCannotCrossConstraint(int, int) const;

    bool GetCannotCrossConstraint(bool, int, int) const;

    void ClearCannotCrossConstraint(int);

    void AddCannotCrossConstraint(bool horiz, int x, int y);  // { cannotCrossEdgeConstraints.push_back({horiz, {x, y}});}
    void AddCannotCrossConstraint(int x, int y);  // { cannotCrossConstraints.push_back({x, y});}
    void AddCannotCrossConstraint(int);  // { cannotCrossConstraints.push_back({x, y});}
    void RemoveCannotCrossConstraint(bool horiz, int x, int y);  // { cannotCrossEdgeConstraints.pop_back();}
    void RemoveCannotCrossConstraint(int x, int y);  // { cannotCrossConstraints.pop_back();}

    void ClearInnerConstraints()
    {
        for (int x = 0; x < width; x++)
        {
            for (int y = 0; y < height; y++)
            {
                regionConstraints[x][y].type = kNoRegionConstraint;
            }
        }
        for (int c = 0; c < kRegionConstraintCount; c++)
            constraintCount[c] = 0;
        constraintCount[kNoRegionConstraint] = width * height;
    }

    void ClearConstraint(WitnessRegionConstraintType t)
    {
        for (int x = 0; x < width; x++)
        {
            for (int y = 0; y < height; y++)
            {
                if (t == regionConstraints[x][y].type)
                {
                    constraintCount[t]--;
                    regionConstraints[x][y].type = kNoRegionConstraint;
                    constraintCount[kNoRegionConstraint]++;
                }
            }
        }
    }

    void RemoveRegionConstraint(int x, int y)
    {
        constraintCount[regionConstraints[x][y].type]--;
        constraintCount[kNoRegionConstraint]++;
        regionConstraints[x][y].type = kNoRegionConstraint;
    }

    /* Triangles - must cross as many edges as triangles */
    static constexpr int GetNumTriangleConstraints() { return width * height; }

    void ClearTriangleConstraints()
    {
        ClearConstraint(kTriangle);
        // triangleConstraints.clear(); triangleConstraints.resize(width*height); triangleCount = 0;
    }

    void AddTriangleConstraint(int x, int y, int count)
    {
        if (x >= width || y >= height || y < 0 || x < 0)
        {
            printf("(%d, %d) out of bounds\n", x, y);
            return;
        }
        assert(count >= 1 && count <= 3);
        constraintCount[regionConstraints[x][y].type]--;
        constraintCount[kTriangle]++;
        regionConstraints[x][y].type = kTriangle;
        regionConstraints[x][y].parameter = count;
        regionConstraints[x][y].color = triangleColor;
    }

    //	{ triangleCount++; triangleConstraints[y*width+x] = count; }
    void AddTriangleConstraint(int which, int count) //{ triangleCount++; triangleConstraints[which] = count; }
    {
        AddTriangleConstraint(which % width, which / width, count);
    }

    /* Color (rounded) square - must separate these */
    /* TODO: Star constraints are a special case */
    void ClearSeparationConstraints() { ClearConstraint(kSeparation); }

    //{ separationConstraints.clear(); separationConstraints.resize(width*height), separationCount = 0; }
    static constexpr int GetNumSeparationConstraints() { return width * height; }

    //	constexpr int GetNumSeparationConstraints() const { return width*height; }
    void AddSeparationConstraint(int x, int y, rgbColor c)
    {
        constraintCount[regionConstraints[x][y].type]--;
        constraintCount[kSeparation]++;
        regionConstraints[x][y].type = kSeparation;
        regionConstraints[x][y].color = c;
    }

    //	{ auto &i = separationConstraints[y*width+x]; i.color = c; i.valid = true; separationCount++;}
    void AddSeparationConstraint(int which, rgbColor c)
    //{ auto &i = separationConstraints[which]; i.color = c; i.valid = true; separationCount++;}
    {
        AddSeparationConstraint(GetRegionFromX(which), GetRegionFromY(which), c);
    }
    //	void RemoveSeparationConstraint(int x, int y) { separationConstraints[y*width+x].valid = false;
    // separationCount--;} 	void RemoveSeparationConstraint(int which) { separationConstraints[which].valid = false;
    // separationCount--;}

    void ClearEraserConstraints() { ClearConstraint(kEraser); }

    //{ separationConstraints.clear(); separationConstraints.resize(width*height), separationCount = 0; }
    constexpr int GetNumEraserConstraints() const { return width * height; }

    void AddEraserConstraint(int x, int y)
    {
        constraintCount[regionConstraints[x][y].type]--;
        constraintCount[kSeparation]++;
        regionConstraints[x][y].type = kEraser;
        regionConstraints[x][y].color = Colors::white;
    }

    //	{ auto &i = separationConstraints[y*width+x]; i.color = c; i.valid = true; separationCount++;}
    void AddEraserConstraint(int which)
    //{ auto &i = separationConstraints[which]; i.color = c; i.valid = true; separationCount++;}
    {
        AddEraserConstraint(GetRegionFromX(which), GetRegionFromY(which));
    }

    // TODO: Not yet complete - don't handle tilted
    /* Tetris constraints - must solve packing problem to validate these */
    /* We allow most constraints with 1...4 blocks -- 14 total */
    /*  1  *
     *  2  **
     *  3  *
     *     *
     *  4  **
     *     *
     *  5  **
     *      *
     *  6  *
     *     **
     *  7   *
     *     **
     *  8  ***
     *  9  *
     *     *
     *     *
     *  10 **
     *     **
     *  11 ***
     *     *
     *  12 *
     *     ***
     *  13 ***
     *       *
     *  14   *
     *     ***
     *  15 **
     *     *
     *     *
     *  16 **
     *      *
     *      *
     *  17 *
     *     *
     *     **
     *  18  *
     *      *
     *     **
     *  19 ****
     *  20 *
     *     *
     *     *
     *     *
     *  21 ***
     *      *
     *  22  *
     *     ***
     *  23 *
     *     **
     *     *
     *  24  *
     *     **
     *      *
     */
    void ClearTetrisConstraints()
    //{ tetrisConstraints.resize(0); tetrisConstraints.resize(width*height); tetrisCount = 0; }
    {
        ClearConstraint(kTetris);
        ClearConstraint(kNegativeTetris);
    }

    void AddNegativeTetrisConstraint(int x, int y, int which)
    {
        assert(which >= 1);
        constraintCount[regionConstraints[x][y].type]--;
        constraintCount[kNegativeTetris]++;
        regionConstraints[x][y].type = kNegativeTetris;
        regionConstraints[x][y].color = tetrisBlue;
        regionConstraints[x][y].parameter = which;
    }

    void AddTetrisConstraint(int x, int y, int which)
    {
        assert(which >= 1);
        constraintCount[regionConstraints[x][y].type]--;
        constraintCount[kTetris]++;
        regionConstraints[x][y].type = kTetris;
        regionConstraints[x][y].color = tetrisYellow;
        regionConstraints[x][y].parameter = which;
    }

    void AddNegativeTetrisConstraint(int loc, int which)
    {
        AddNegativeTetrisConstraint(GetRegionFromX(loc), GetRegionFromY(loc), which);
    }

    void AddTetrisConstraint(int loc, int which) // { tetrisConstraints[loc] = which; tetrisCount++; }
    {
        AddTetrisConstraint(GetRegionFromX(loc), GetRegionFromY(loc), which);
    }

    static constexpr int GetNumTetrisConstraints() { return width * height; }

    // TODO: don't increase count if piece is already there
    const uint16_t tetrisBits[25] = {0, 0x8000, 0xC000, 0x8800, 0xC800, 0xC400, 0x8C00, 0x4C00, 0xE000, 0x8880, 0xCC00,
                                     0xE800, 0x8E00, 0xE200, 0x2E00, 0xC880, 0xC440, 0x88C0, 0x44C0, 0xF000, 0x8888,
                                     0xE400, 0x4E00, 0x8C80, 0x4C40};
    const uint64_t tetrisBits64[25] = {0, 0x8000000000000000ull, 0xC000000000000000ull, 0x8080000000000000ull,
                                       0x80C0000000000000ull, 0x40C0000000000000ull, 0xC080000000000000ull,
                                       0xC040000000000000ull,
                                       0xE000000000000000ull, 0x8080800000000000ull, 0xC0C0000000000000ull,
                                       0x80E0000000000000ull,
                                       0xE080000000000000ull, 0x20E0000000000000ull, 0xE020000000000000ull,
                                       0x8080C00000000000ull,
                                       0x4040C00000000000ull, 0xC080800000000000ull, 0xC040400000000000ull,
                                       0xF000000000000000ull,
                                       0x8080808000000000ull, 0x40E0000000000000ull, 0xE040000000000000ull,
                                       0x80C0800000000000ull,
                                       0x40C0400000000000ull};
    const uint16_t tetrisSize[25] = {0, 1, 2, 2, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4};
    const uint16_t tetrisWH[25][2] = {{0, 0},
                                      {1, 1},
                                      {2, 1},
                                      {1, 2},
                                      {2, 2},
                                      {2, 2},
                                      {2, 2},
                                      {2, 2},
                                      {3, 1},
                                      {1, 3},
                                      {2, 2},
                                      {3, 2},
                                      {3, 2},
                                      {3, 2},
                                      {3, 2},
                                      {2, 3},
                                      {2, 3},
                                      {2, 3},
                                      {2, 3},
                                      {4, 1},
                                      {1, 4},
                                      {3, 2},
                                      {3, 2},
                                      {2, 3},
                                      {2, 3}};

    void ClearStarConstraints() { ClearConstraint(kStar); }

    static constexpr int GetNumStarConstraints() { return width * height; }

    void AddStarConstraint(int x, int y, rgbColor c)
    {
        if (x >= width || y >= height || y < 0 || x < 0)
        {
            printf("(%d, %d) out of bounds\n", x, y);
            return;
        }
        constraintCount[regionConstraints[x][y].type]--;
        constraintCount[kStar]++;
        regionConstraints[x][y].type = kStar;
        regionConstraints[x][y].color = c;
    }

    void AddStarConstraint(int which, rgbColor c)
    {
        AddStarConstraint(GetRegionFromX(which), GetRegionFromY(which), c);
    }

    void AddRegionConstraint(int x, int y, const WitnessRegionConstraint &constraint)
    {
        switch (constraint.type)
        {
            case kSeparation:
                AddSeparationConstraint(x, y, constraint.color);
                break;
            case kStar:
                AddStarConstraint(x, y, constraint.color);
                break;
            case kTetris:
                AddTetrisConstraint(x, y, constraint.parameter);
                break;
            case kNegativeTetris:
                AddNegativeTetrisConstraint(x, y, constraint.parameter);
                break;
            case kTriangle:
                AddTriangleConstraint(x, y, constraint.parameter);
                break;
            case kEraser:
                AddEraserConstraint(x, y);
                break;
            default: // kNoRegionConstraint, kRegionConstraintCount
                break;
        }
    }

    void AddRegionConstraint(int which, const WitnessRegionConstraint &constraint)
    {
        AddRegionConstraint(GetRegionFromX(which), GetRegionFromY(which), constraint);
    }

    std::vector<std::pair<int, int>> start;
    std::vector<std::pair<int, int>> goal;
    // constant-time lookup for which goal is nearby
    // value is hte index in the goal array+1 (0 index means no goal)
    std::array<int, (width + 1) * (height + 1)> goalMap;
    //	const int kStartX = 0, kStartY = 0;

    void BuildLocationMap(std::array<std::pair<unsigned, unsigned>,
            (GetNumPathConstraints() + width * height)> &map) const
    {
        for (auto y = height; y >= 0; --y)
        {
            for (auto x = 0; x <= width; ++x)
            {
                auto p = width * (height + 1) + (width + 1) * height + (width + 1) * y + x;
                auto loc = (height - y) * (width * 2 + 1) * 2 + x * 2;
                map[loc] = std::pair<unsigned, unsigned>(0, p);
            }
        }
        for (auto y = height - 1; y >= 0; --y)
        {
            for (auto x = 0; x <= width; ++x)
            {
                auto p = width * (height + 1) + x * height + y;
                auto loc = ((height - y) * 2 - 1) * (width * 2 + 1) + x * 2;
                map[loc] = std::pair<unsigned, unsigned>(0, p);
            }
        }
        for (auto y = height; y >= 0; --y)
        {
            for (auto x = 0; x < width; ++x)
            {
                auto p = width * y + x;
                auto loc = (height - y) * (width * 2 + 1) * 2 + x * 2 + 1;
                map[loc] = std::pair<unsigned, unsigned>(0, p);
            }
        }
        for (auto y = height - 1; y >= 0; --y)
        {
            for (auto x = 0; x < width; ++x)
            {
                auto p = width * y + x;
                auto loc = ((height - y) * 2 - 1) * (width * 2 + 1) + x * 2 + 1;
                map[loc] = std::pair<unsigned, unsigned>(1, p);
            }
        }
        for (auto [x, y]: start)
        {
            auto p = width * (height + 1) + (width + 1) * height + (width + 1) * y + x;
            auto loc = (height - y) * (width * 2 + 1) * 2 + x * 2;
            map[loc] = std::pair<unsigned, unsigned>(2, p);
        }
        for (auto [x, y]: goal)
        {
            if (x > width)
                x = width;
            if (x < 0)
                x = 0;
            if (y > height)
                y = height;
            if (y < 0)
                y = 0;
            auto p = width * (height + 1) + (width + 1) * height + (width + 1) * y + x;
            auto loc = (height - y) * (width * 2 + 1) * 2 + x * 2;
            map[loc] = std::pair<unsigned, unsigned>(3, p);
        }
    }

    /**
     * Simulate the conversion from the protobuf of The Windmill to JSON
     * @see https://github.com/thefifthmatt/windmill-client/blob/master/src/grid.js
     * @see https://github.com/thefifthmatt/windmill-client/blob/master/src/grid.proto
     */
    explicit operator std::string() const
    {
        std::stringstream ss;
        ss << "{" << std::quoted("width") << ": " << width * 2 + 1 << ", "
            << std::quoted("symmetry") << ": 0" << ", "
            << std::quoted("entity") << ": [";
        std::array<std::pair<unsigned, unsigned>, (GetNumPathConstraints() + width * height)> locationMap;
        BuildLocationMap(locationMap);
        for (auto i = 0; i < locationMap.size(); ++i)
        {
            auto [type, location] = locationMap[i];
            std::stringstream ess;
            switch (type) {
                case 0: // path constraints
                    switch (pathConstraints[location]) {
                        case kCannotCross:
                        {
                            ess << "{" << std::quoted("type") << ": 5}";
                            break;
                        }
                        case kMustCross:
                        {
                            ess << "{" << std::quoted("type") << ": 6}";
                            break;
                        }
                        default:
                            ess << "{}";
                            break;
                    }
                    break;
                case 1: // region constrains
                    ess << std::string(GetRegionConstraint(GetRegionFromX(location),
                                                       GetRegionFromY(location)));
                    break;
                case 2: // start
                    ess << "{" << std::quoted("type") << ": 3}";
                    break;
                case 3: // end
                    ess << "{" << std::quoted("type") << ": 4}";
                    break;
                default:
                    break;
            }
            ss << ess.str();
            if (i < locationMap.size() - 1)
                ss << ",";
        }
        ss << "]}";
        return ss.str();
    }

    std::ostream& Serialize(std::ostream &os) const
    {
        return os << std::string(*this);
    }

    std::istream& Deserialize(std::istream &is)
    {
        std::string input((std::istreambuf_iterator<char>(is)), std::istreambuf_iterator<char>());
        std::regex w_r(R"("width":\s*(\d+))");
        std::regex e_r(R"("entity":\s*\[(.*)\])");
        std::regex s_r(R"("symmetry":\s*(\d+))");
        std::smatch w_match, e_match, s_match;
        if (!(std::regex_search(input, w_match, w_r) &&
              std::regex_search(input, s_match, s_r) &&
              std::regex_search(input, e_match, e_r)))
            throw std::invalid_argument("incorrect string");

        if (std::stoi(w_match[1].str()) != (width * 2 + 1))
            throw std::invalid_argument("unsupported size");

        if (std::stoi(s_match[1].str()) != 0)
            throw std::invalid_argument("unsupported symmetry");

        std::array<std::pair<unsigned, unsigned>, (GetNumPathConstraints() + width * height)> locationMap;
        BuildLocationMap(locationMap);
        auto es = std::string(R"(\{"type":\s*(\d+),\s*"color":\s*(\d+),\s*)") +
                  std::string(R"("orientation":\s*(?:null|\{"horizontal":\s*\d,\s*"vertical":\s*\d\}),\s*)") +
                  std::string(R"("shape":\s*(null|\{"width":\s*(\d),\s*)") +
                  std::string(R"("grid":\s*(\[(?:(?:true|false)\s*,\s*)*(?:true|false)?\]),\s*"free":\s*(true|false),\s*)") +
                  std::string(R"("negative":\s*(true|false)\}),\s*"count":\s*(\d+),\s*"triangle_count":\s*(\d+)\})");
        std::regex es_r(es);
        auto entities = e_match[1].str();
        unsigned count = 0;
        for (auto i = std::sregex_iterator(entities.begin(), entities.end(), es_r); i != std::sregex_iterator(); ++i)
        {
            const auto& match = *i;
            int type = std::stoi(match[1].str());
            switch (type) {
                case 3:
                case 4: // only support single start and goal currently
                {
                    ++count;
                    break;
                }
                case 5:
                case 6:
                {
                    auto loc = locationMap[count++].second;
                    (type == 5) ? SetCannotCrossConstraint(loc) : SetMustCrossConstraint(loc);
                    break;
                }
                case 7:
                {
                    auto loc = locationMap[count++].second;
                    auto color = GetColorFromEnum(std::stoi(match[2].str()));
                    AddSeparationConstraint(loc, (color == Colors::white) ? Colors::lightgray : color);
                    break;
                }
                case 8:
                {
                    auto loc = locationMap[count++].second;
                    auto color = GetColorFromEnum(std::stoi(match[2].str()));
                    AddStarConstraint(loc, (color == Colors::white) ? Colors::lightgray : color);
                    break;
                }
                case 9:
                {
                    auto loc = locationMap[count++].second;
                    int param = GetTetrisParameterFromString(std::stoi(match[4].str()), match[5].str());
                    if (param >= 1)
                        (match[7].str() == "false") ? AddTetrisConstraint(loc, param) :
                            AddNegativeTetrisConstraint(loc, param);
                    break;
                }
                case 11:
                {
                    auto loc = locationMap[count++].second;
                    AddTriangleConstraint(loc, std::stoi(match[9].str()));
                    break;
                }
                case 0:
                case 1:
                case 2:
                case 10:
                default: // empty
                {
                    int c = std::stoi(match[8].str());
                    if (c == 0)
                        c = 1;
                    count += c;
                    break;
                }
            }
        }
        return is;
    }

    std::string SaveToHashString() const
    {
        // JSON string { "name":"value" }
        // JSON number { "name":number }
        // JSON array { "name":{ "object", "object", "object"} }
        std::string quote = "\"";
        std::string hash = "{";

        {
            hash += quote + "dim" + quote + ":" + quote + std::to_string(width) + "x" + std::to_string(height) + quote;
            hash += ",";
        }

		{
            hash += quote + "cc" + quote + ":{";
            for (int x = 0; x < width; x++)
            {
                for (int y = 0; y < height; y++)
                {
                    if (x != 0 || y != 0) hash += ",";
                    hash += quote;
                    hash += std::to_string(regionConstraints[x][y].t) + ";";
                    // no point writing garbage
                    if (regionConstraints[x][y].t == kNoRegionConstraint)
                        hash += "0;";
                    else
                        hash += std::to_string(regionConstraints[x][y].parameter) + ";";
					if (regionConstraints[x][y].t == kNoRegionConstraint)
						hash += "#DADFAD";
					else
						hash += regionConstraints[x][y].c.hex();
                    hash += quote;
                }
            }
            hash += "},";
        }

        {
            hash += quote + "mc" + quote + ":" + quote;
            // add must-cross constraints
            for (int x = 0; x < GetNumPathConstraints(); x++)
                hash += GetMustCrossConstraint(x) ? "1" : (GetCannotCrossConstraint(x) ? "2" : "0");
            hash += quote;
        }
        hash += "}";
        return hash;
    }

    void GetDimensionsFromHashString(const std::string &s, int &w, int &h) const
    {
        const char *loc = strstr(s.c_str(), "dim");
        if (loc == nullptr)
        {
            w = -1;
            h = -1;
            return;
        }
        char a, b;
        sscanf(loc, "%*[^0-9]%cx%c", &a, &b);
        w = a - '0';
        h = b - '0';
        printf("Width: %d; height: %d\n", w, h);
    }

    // TODO: Code assumes input is well-formed! Fix this.
    void LoadFromHashString(std::string s)
    {
        Reset();
        // looking for dim, cc & mc
        int w, h;
        GetDimensionsFromHashString(s, w, h);
        if (w != width || h != height)
        {
            printf("Dimensions do not match, aborting load!\n");
            return;
        }

        // get crossing constraints
        {
            const char *loc = strstr(s.c_str(), "mc");
            if (loc == nullptr)
            {
                printf("Failed loading MC constraints\n");
                return;
            }
            // get end of mc variable
            while (loc[0] != ':')
                loc++;
            // get start of mc constraints
            while (loc[0] != '"')
                loc++;
            loc++;
            for (int x = 0; x < GetNumPathConstraints(); x++)
            {
                if (loc[0] == '1')
                    SetMustCrossConstraint(x);
                else if (loc[0] == '2')
                    SetCannotCrossConstraint(x);
                loc++;
            }
        }

        // get regular constraints
        {
            const char *loc = strstr(s.c_str(), "cc");
            if (loc == nullptr)
            {
                printf("Failed loading CC constraints\n");
                return;
            }
            // get end of mc variable
            while (loc[0] != '{')
                loc++;

            for (int x = 0; x < width; x++)
            {
                for (int y = 0; y < height; y++)
                {
                    if (loc[0] == '}')
                    {
                        printf("Unexpected '}', aborting\n");
                        return;
                    }
                    while (loc[0] != '"')
                        loc++;

                    loc++; // skip open quote
                    int cType = atoi(loc);
                    while (loc[0] != ';')
                        loc++;
                    loc++;
                    int param = atoi(loc);
                    while (loc[0] != ';')
                        loc++;
                    loc++;
                    rgbColor c;
                    c.hex(loc);

                    constraintCount[regionConstraints[x][y].type]--;
                    regionConstraints[x][y].type = static_cast<WitnessRegionConstraintType>(cType);
                    constraintCount[regionConstraints[x][y].type]++;
                    regionConstraints[x][y].color = c;
                    regionConstraints[x][y].parameter = param;

                    while (loc[0] != ',')
                        loc++;
                }
            }
        }
    }

    const rgbColor tetrisYellow = {0.862745098f, 0.6549019608f, 0.0f};
    const rgbColor tetrisBlue = {0.2196078431f, 0.3607843137f, 0.8705882353f};
    const rgbColor drawColor = Colors::darkbluegray; // Colors::lightblue;
    const rgbColor lineColor = Colors::lightgray;
    const rgbColor backColor = Colors::white; // Colors::gray;
    const rgbColor outerBackColor = Colors::white;
    const rgbColor triangleColor = Colors::orange;

    float scale = 0.75f;
    float gapOffset = scale * 2.0f / (width > height ? width : height); // size of each square
    float lineWidth = gapOffset * 0.1f;
    float xGap = (((width > height) ? (width) : (height)) - width) * gapOffset / 2.0f;
    float yGap = -(((width > height) ? (width) : (height)) - height) * gapOffset / 2.0f;

    int GetPathIndex(int x, int y) const { return y * (width + 1) + x; }

    struct PathLocation
    {
        unsigned t;
        int x;
        int y;
    };

    PathLocation GetPathLocation(int index) const
    {
        for (int x = 0; x < width; x++)
            for (int y = 0; y <= height; y++)
                if (index == (x + y * width))
                    return {0, x, y};
        for (int x = 0; x <= width; x++)
            for (int y = 0; y < height; y++)
                if (index == (width * (height + 1) + x * height + y))
                    return {1, x, y};
        for (int x = 0; x < width + 1; x++)
            for (int y = 0; y < height + 1; y++)
                if (index == (width * (height + 1) + (width + 1) * height + (width + 1) * y + x))
                    return {2, x, y};
        return {};
    }

    int GetRegionIndex(int x, int y) const { return y * width + x; }

    int GetRegionFromX(int index) const { return index % width; }

    int GetRegionFromY(int index) const { return index / width; }

    void LabelRegions(const WitnessState<width, height> &s) const;

    // this is mapped to a 2d map which has the region number for any x/y location
    // mutable std::vector<int> regions;
    mutable std::array<int, width * height> regions;
    // for each region, this is a list of the cells in the region
    // the size of the vector tells us the size of the region
    mutable std::vector<std::vector<int> *> regionList;
    mutable vectorCache<int> regionCache;

    mutable std::vector<int> tetrisBlockCount;
    mutable std::vector<int> tetrisBlocksInRegion;

    bool CheckPathConstraintsForRegion(int region, const WitnessState<width, height> &state) const
    {
        int x = GetRegionFromX(region);
        int y = GetRegionFromY(region);

        // vertices
        if (GetMustCrossConstraint(x, y) && (!state.Occupied(x, y)))
            return false;
        if (GetCannotCrossConstraint(x, y) && (state.Occupied(x, y)))
            return false;
        if (GetMustCrossConstraint(x + 1, y) && (!state.Occupied(x + 1, y)))
            return false;
        if (GetCannotCrossConstraint(x + 1, y) && (state.Occupied(x + 1, y)))
            return false;
        if (GetMustCrossConstraint(x, y + 1) && (!state.Occupied(x, y + 1)))
            return false;
        if (GetCannotCrossConstraint(x, y + 1) && (state.Occupied(x, y + 1)))
            return false;
        if (GetMustCrossConstraint(x + 1, y + 1) && (!state.Occupied(x + 1, y + 1)))
            return false;
        if (GetCannotCrossConstraint(x + 1, y + 1) && (state.Occupied(x + 1, y + 1)))
            return false;

        // horizontal edges
        if (GetMustCrossConstraint(true, x, y) && !state.OccupiedEdge(x, y, x + 1, y))
            return false;
        if (GetCannotCrossConstraint(true, x, y) && state.OccupiedEdge(x, y, x + 1, y))
            return false;
        if (GetMustCrossConstraint(true, x, y + 1) && !state.OccupiedEdge(x, y + 1, x + 1, y + 1))
            return false;
        if (GetCannotCrossConstraint(true, x, y + 1) && state.OccupiedEdge(x, y + 1, x + 1, y + 1))
            return false;

        // vertical edges
        if (GetMustCrossConstraint(false, x, y) && !state.OccupiedEdge(x, y, x, y + 1))
            return false;
        if (GetCannotCrossConstraint(false, x, y) && state.OccupiedEdge(x, y, x, y + 1))
            return false;
        if (GetMustCrossConstraint(false, x + 1, y) && !state.OccupiedEdge(x + 1, y, x + 1, y + 1))
            return false;
        if (GetCannotCrossConstraint(false, x + 1, y) && state.OccupiedEdge(x + 1, y, x + 1, y + 1))
            return false;

        return true;
    }

    bool RecursivelyPlacePieces(
            int curr, uint64_t board, uint64_t oob, uint64_t posFootprint, uint64_t negFootprint) const;

    void GetMouseActions(const WitnessState<width, height> &nodeID, std::vector <WitnessAction> &actions) const;

    void DoLine(
            Graphics::Display &display, const Graphics::point &p1, const Graphics::point &p2, const rgbColor &c) const
    {
        if (p1.x == p2.x) // vertical line
        {
            display.FillRect({(p1.x - lineWidth), std::min(p1.y, p2.y), (p1.x + lineWidth), std::max(p1.y, p2.y)}, c);
        }
        else if (p1.y == p2.y) // horizontal line
        {
            display.FillRect({std::min(p1.x, p2.x), (p1.y - lineWidth), std::max(p1.x, p2.x), (p1.y + lineWidth)}, c);
        }
        else
        { // other line
            display.DrawLine(p1, p2, 2 * lineWidth, c);
        }
    }

    Graphics::point GetScreenCoord(int x, int y) const
    {
        float xMargin = 0;
        float yMargin = 0;
        if (x > width && y > height) // upper right corner
        {
            x = width;
            y = height;
            xMargin = 2 * lineWidth;
            yMargin = -2 * lineWidth;
        }
        if (y > height)
        {
            y = height;
            yMargin = -2 * lineWidth;
            if (x > width) x = width;
            if (x < 0) x = 0;
        }
        else if (x > width)
        {
            xMargin = 2 * lineWidth;
            x = width;
            if (y < 0) y = 0;
        }
        if (x < 0)
        {
            xMargin = -2 * lineWidth;
            x = 0;
            if (y < 0) y = 0;
        }
        else if (y < 0)
        {
            yMargin = 2 * lineWidth;
            y = 0;
        }

        //		if (x > width || y > height)
        //			return {(-scale+x*gapOffset+xGap), (scale-y*gapOffset+yGap)};
        //			return {(-scale+width*gapOffset+xGap), (scale-height*gapOffset+yGap-2*lineWidth)};
        //			return {(-scale+width*gapOffset+xGap), (scale-(height+4*lineWidth)*gapOffset+yGap)};

        return Graphics::point{(-scale + x * gapOffset + xGap + xMargin), (scale - y * gapOffset + yGap + yMargin)};
    }

    void DebugPrint(uint64_t val, int offset = 0) const
    {
        for (int y = 7; y >= 0; y--)
        {
            for (int x = 0; x < offset; x++)
                std::cout << " ";
            for (int x = 0; x < 8; x++)
            {
                std::cout << ((val >> (7 - x)) >> ((7 - y) * 8) & 0x1);
            }
            std::cout << std::endl;
        }
    }
};

template<int width, int height>
void Witness<width, height>::GetSuccessors(
        const WitnessState<width, height> &nodeID, std::vector <WitnessState<width, height>> &neighbors) const
{
}

template<int width, int height>
void Witness<width, height>::GetActions(
        const WitnessState<width, height> &nodeID, std::vector<WitnessAction> &actions) const
{
    actions.resize(0);
    if (nodeID.path.size() == 0)
    {
        actions.push_back(kStart);
        return;
    }

    int currX = nodeID.path.back().first;
    int currY = nodeID.path.back().second;

    if (currX > width || currY > height) return;

    // TODO: Only works with one exit going from lower left to upper right
    if (goalMap[GetPathIndex(currX, currY)] != 0) actions.push_back(kEnd);
    //	for (const auto &ends : goal)
    //	{
    //		if (currX == ends.first && currY == ends.second)
    //		{
    //			actions.push_back(kEnd);
    //			break;
    //		}
    //	}
    //	if (currX == width && currY == height)
    //	{
    //		actions.push_back(kEnd);
    //		return;
    //	}

    if (currX > 0 && !nodeID.Occupied(currX - 1, currY)) actions.push_back(kLeft);
    if (currX < width && !nodeID.Occupied(currX + 1, currY)) actions.push_back(kRight);
    if (currY > 0 && !nodeID.Occupied(currX, currY - 1)) actions.push_back(kDown);
    if (currY < height && !nodeID.Occupied(currX, currY + 1)) actions.push_back(kUp);
}

template<int width, int height>
void Witness<width, height>::GetMouseActions(
        const WitnessState<width, height> &nodeID, std::vector<WitnessAction> &actions) const
{
    actions.resize(0);
    if (nodeID.path.size() == 0)
    {
        actions.push_back(kStart);
        return;
    }

    int currX = nodeID.path.back().first;
    int currY = nodeID.path.back().second;

    if (!nodeID.InGoal())
    {
        // could be next to a goal
        if (goalMap[(GetPathIndex(currX, currY))] != 0)
        {
            actions.push_back(kEnd);
        }
    }
    //	if (currX == width && currY == height)
    //		actions.push_back(kEnd);
    //	if (currX > width || currY > height)
    //	{
    //		actions.push_back(kDown);
    //		actions.push_back(kEnd);
    //		return;
    //	}

    //	bool noLeft = false, noRight = false, noUp = false, noDown = false;
    //	for (auto &i : cannotCrossConstraints)
    //	{
    //		if (i.first == currX-1 && i.second == currY)
    //			noLeft = true;
    //		if (i.first == currX+1 && i.second == currY)
    //			noRight = true;
    //		if (i.second == currY-1 && i.first == currX)
    //			noDown = true;
    //		if (i.second == currY+1 && i.first == currX)
    //			noUp = true;
    //	}
    //	if (currX > 0 && !noLeft)
    //		actions.push_back(kLeft);
    //	if (currX < width && !noRight)
    //		actions.push_back(kRight);
    //	if (currY > 0 && !noDown)
    //		actions.push_back(kDown);
    //	if (currY < height && !noUp)
    //		actions.push_back(kUp);
    if (currY >= 0 && currY <= height)
    {
        if (currX > 0 && !GetCannotCrossConstraint(currX - 1, currY)) actions.push_back(kLeft);
        if (currX < width && !GetCannotCrossConstraint(currX + 1, currY)) actions.push_back(kRight);
    }
    if (currX >= 0 && currX <= width)
    {
        if (currY > 0 && !GetCannotCrossConstraint(currX, currY - 1)) actions.push_back(kDown);
        if (currY < height && !GetCannotCrossConstraint(currX, currY + 1)) actions.push_back(kUp);
    }
}

template<int width, int height>
void Witness<width, height>::ApplyAction(std::pair<int, int> &s, WitnessAction a) const
{
    switch (a)
    {
    case kEnd:
    {
        int whichGoal = goalMap[GetPathIndex(s.first, s.second)];
        assert(whichGoal != 0);
        s = goal[whichGoal - 1];
    }
        //			//s.first++;
        //			if (s.first == width)
        //				s.first++;
        //			else if (s.first == 0)
        //				s.first--;
        //			if (s.second == height)
        //				s.second++;
        //			else if (s.second == 0)
        //				s.second--;
        break;
    case kStart:
        break;
    case kLeft:
        s.first--;
        break;
    case kRight:
        s.first++;
        break;
    case kUp:
        s.second++;
        break;
    case kDown:
        s.second--;
        break;
    }
}

template<int width, int height>
void Witness<width, height>::ApplyAction(WitnessState<width, height> &s, WitnessAction a) const
{
    auto len = s.path.size() + 1;
    switch (a)
    {
    case kEnd:
    {
        int whichGoal = goalMap[GetPathIndex(s.path.back().first, s.path.back().second)];
        assert(whichGoal != 0);
        s.path.push_back(goal[whichGoal - 1]);
    }
        //			if (s.path.back().first >= width)
        //				s.path.push_back({width+1, s.path.back().second});
        ////				s.first++;
        //			else if (s.path.back().first <= 0)
        //				s.path.push_back({-1, s.path.back().second});
        //			//			s.first--;
        //			else if (s.path.back().second >= height)
        //				s.path.push_back({s.path.back().first, height+1});
        ////				s.second++;
        //			else if (s.path.back().second <= 0)
        //				s.path.push_back({s.path.back().first, -1});
        ////				s.second--;
        //			else {
        //				assert(false);
        //				//s.path.push_back({width, height+1});
        //			}
        return; // don't occupy on end action
    case kStart:
        s.Reset();
        s.path.push_back(start[0]);
        s.Occupy(s.path.back().first, s.path.back().second);
        break;
    case kLeft:
        s.path.push_back(s.path.back());
        s.path.back().first--;
        s.Occupy(s.path.back().first, s.path.back().second);
        s.OccupyEdge(s.path[len - 1].first, s.path[len - 1].second, s.path[len - 2].first, s.path[len - 2].second);
        break;
    case kRight:
        s.path.push_back(s.path.back());
        s.path.back().first++;
        s.Occupy(s.path.back().first, s.path.back().second);
        s.OccupyEdge(s.path[len - 1].first, s.path[len - 1].second, s.path[len - 2].first, s.path[len - 2].second);
        break;
    case kUp:
        s.path.push_back(s.path.back());
        s.path.back().second++;
        s.Occupy(s.path.back().first, s.path.back().second);
        s.OccupyEdge(s.path[len - 1].first, s.path[len - 1].second, s.path[len - 2].first, s.path[len - 2].second);
        break;
    case kDown:
        s.path.push_back(s.path.back());
        s.path.back().second--;
        s.Occupy(s.path.back().first, s.path.back().second);
        s.OccupyEdge(s.path[len - 1].first, s.path[len - 1].second, s.path[len - 2].first, s.path[len - 2].second);
        break;
    }

    // s.occupiedCorners.set(s.path.back().second*width+s.path.back().first); // y*width+x;
}

template<int width, int height>
void Witness<width, height>::UndoAction(WitnessState<width, height> &s, WitnessAction a) const
{
    auto len = s.path.size();
    if (s.path[len - 1].first <= width && s.path[len - 1].second <= height)
    {
        if (len > 1)
            s.UnoccupyEdge(
                    s.path[len - 1].first, s.path[len - 1].second, s.path[len - 2].first, s.path[len - 2].second);
        s.Unoccupy(s.path.back().first, s.path.back().second);
    }
    s.path.pop_back();
}

template<int width, int height>
bool Witness<width, height>::Legal(WitnessState<width, height> &s, WitnessAction a, legality &l) const
{
    int currX = s.path.back().first;
    int currY = s.path.back().second;
    switch (a)
    {
    case kEnd:
        if (goalMap[GetPathIndex(currX, currY)] != 0)
        {
            l = kLegal;
            return true;
        }
        l = kNotAtEnd;
        return false;
        //			for (const auto &i : goal)
        //				if (i.first == currX && i.second == currY)
        //				{
        //					l = kLegal;
        //					return true;
        //				}
        ////			if (currX == width && currY == height)
        ////			{
        ////				l = kLegal;
        ////				return true;
        ////			}
        //			l = kNotAtEnd;
        //			return false;
    case kStart:
        if (s.path.size() == 0)
        {
            l = kLegal;
            return true;
        }
        l = kNotAtStart;
        return false;
    case kLeft:
        //			if (currX > 0 && !s.Occupied(currX-1, currY) && !GetCannotCrossConstraint(true, currX-1, currY))
        if (currX <= 0)
        {
            l = kNotValidAction;
            return false;
        }
        else if (GetCannotCrossConstraint(true, currX - 1, currY))
        {
            l = kHitCannotCross;
            return false;
        }
        else if (s.Occupied(currX - 1, currY))
        {
            l = kHitLine;
            return false;
        }
        l = kLegal;
        return true;
    case kRight:
        if (currX >= width)
        {
            l = kNotValidAction;
            return false;
        }
        else if (GetCannotCrossConstraint(true, currX, currY))
        {
            l = kHitCannotCross;
            return false;
        }
        else if (s.Occupied(currX + 1, currY))
        {
            l = kHitLine;
            return false;
        }
        l = kLegal;
        return true;
        //			return (currX < width && !s.Occupied(currX+1, currY) && !GetCannotCrossConstraint(true, currX,
        // currY));
    case kDown:
        // return (currY > 0 && !s.Occupied(currX, currY-1) && !GetCannotCrossConstraint(false, currX, currY-1));
        if (currY <= 0)
        {
            l = kNotValidAction;
            return false;
        }
        else if (GetCannotCrossConstraint(false, currX, currY - 1))
        {
            l = kHitCannotCross;
            return false;
        }
        else if (s.Occupied(currX, currY - 1))
        {
            l = kHitLine;
            return false;
        }
        l = kLegal;
        return true;
    case kUp:
        //			return (currY < height && !s.Occupied(currX, currY+1) && !GetCannotCrossConstraint(false, currX,
        // currY));
        if (currY >= height)
        {
            l = kNotValidAction;
            return false;
        }
        else if (GetCannotCrossConstraint(false, currX, currY))
        {
            l = kHitCannotCross;
            return false;
        }
        else if (s.Occupied(currX, currY + 1))
        {
            l = kHitLine;
            return false;
        }
        l = kLegal;
        return true;
    }
    return false;
}

template<int width, int height>
bool Witness<width, height>::Legal(WitnessState<width, height> &s, WitnessAction a) const
{
    int currX = s.path.back().first;
    int currY = s.path.back().second;
    switch (a)
    {
    case kEnd:
        if (goalMap[GetPathIndex(currX, currY)] != 0)
        {
            return true;
        }
        return false;
        //			for (const auto &i : goal)
        //				if (i.first == currX && i.second == currY)
        //				{
        //					return true;
        //				}
        //			return false;
    case kStart:
        return s.path.size() == 0;
    case kLeft:
        return (currX > 0 && !s.Occupied(currX - 1, currY) && !GetCannotCrossConstraint(true, currX - 1, currY));
    case kRight:
        return (currX < width && !s.Occupied(currX + 1, currY) && !GetCannotCrossConstraint(true, currX, currY));
    case kDown:
        return (currY > 0 && !s.Occupied(currX, currY - 1) && !GetCannotCrossConstraint(false, currX, currY - 1));
    case kUp:
        return (currY < height && !s.Occupied(currX, currY + 1) && !GetCannotCrossConstraint(false, currX, currY));
    }
    return false;
}

template<int width, int height>
bool Witness<width, height>::InvertAction(WitnessAction &a) const
{
    switch (a)
    {
    case kStart:
        return false;
    case kEnd:
        return false;
    case kUp:
        a = kDown;
        break;
    case kDown:
        a = kUp;
        break;
    case kLeft:
        a = kRight;
        break;
    case kRight:
        a = kLeft;
        break;
    }
    return true;
}

/** Heuristic value between two arbitrary nodes. **/
template<int width, int height>
double Witness<width, height>::HCost(
        const WitnessState<width, height> &node1, const WitnessState<width, height> &node2) const
{
    return 0;
}

template<int width, int height>
double Witness<width, height>::GCost(
        const WitnessState<width, height> &node1, const WitnessState<width, height> &node2) const
{
    return 1;
}

template<int width, int height>
double Witness<width, height>::GCost(const WitnessState<width, height> &node, const WitnessAction &act) const
{
    return 1;
}

template<int width, int height>
bool Witness<width, height>::GoalTest(
        const WitnessState<width, height> &node, const WitnessState<width, height> &goal) const
{
    // Check constraints
    return GoalTest(node);
}

template<int width, int height>
bool Witness<width, height>::RegionTest(const WitnessState<width, height> &node) const
{
    LabelRegions(node);
    for (auto i = 0; i < regionList.size(); ++i)
    {
        const auto &region = *regionList[i];
        if (region.empty())
            continue;
        if (std::find(region.begin(), region.end(), width * height - 1) != region.end()) // skip the goal
            continue;
        bool found = false;
        rgbColor c;
        tetrisBlockCount.resize(regionList.size());
        tetrisBlockCount[i] = 0;
        for (auto r: region)
        {
            if (!CheckPathConstraintsForRegion(r, node))
                return false;

            int x = GetRegionFromX(r);
            int y = GetRegionFromY(r);
            rgbColor finishedColor(1.0 / 512.0, 1.0 / 512.0, 1.0 / 512.0);
            const auto &constraint = regionConstraints[x][y];
            switch (constraint.type) {
                case kTriangle:
                {
                    if (node.OccupiedEdge(x, y, x, y + 1) +
                        node.OccupiedEdge(x, y, x + 1, y) +
                        node.OccupiedEdge(x + 1, y, x + 1, y + 1) +
                        node.OccupiedEdge(x, y + 1, x + 1, y + 1)
                        != constraint.parameter)
                        return false;
                    break;
                }
                case kSeparation:
                {
                    if (!found)
                    {
                        c = constraint.color;
                        found = true;
                    }
                    else if (c != constraint.color)
                        return false;
                    break;
                }
                case kStar:
                {
                    if (constraint.color == finishedColor)
                        continue;
                    finishedColor = constraint.color;
                    int count = 0;
                    for (auto rr: region)
                    {
                        int xx = GetRegionFromX(rr);
                        int yy = GetRegionFromY(rr);

                        if (regionConstraints[xx][yy].type != kNoRegionConstraint &&
                            constraint.color == regionConstraints[xx][yy].color)
                        {
                            count++;
                            if (count > 2)
                                return false;
                        }
                    }
                    if (count != 2)
                        return false;
                }
                case kTetris:
                {
                    tetrisBlockCount[i] += tetrisSize[constraint.parameter];
                    break;
                }
                case kNegativeTetris:
                {
                    tetrisBlockCount[i] -= tetrisSize[constraint.parameter];
                    break;
                }
                default:
                    break;
            }
            if (tetrisBlockCount[i] > 0 && tetrisBlockCount[i] != region.size())
                return false;
        }
    }
    if (constraintCount[kTetris] > 0)
    {
        for (auto region: regionList)
        {
            if (region->empty())
                continue;
            if (std::find(region->begin(), region->end(), width * height - 1) != region->end())
                continue;
            bool hasNegations = false;
            tetrisBlocksInRegion.resize(0);
            uint64_t board = 0;
            for (auto r: *region)
            {
                uint64_t x = GetRegionFromX(r);
                uint64_t y = GetRegionFromY(r);

                board |= ((1ull << (7 - x)) << ((7 - y) * 8));

                const auto &constraint = regionConstraints[x][y];
                if (constraint.type == kTetris)
                    tetrisBlocksInRegion.push_back(constraint.parameter);
                if (constraint.type == kNegativeTetris)
                {
                    tetrisBlocksInRegion.push_back(-constraint.parameter);
                    hasNegations = true;
                }
            }

            if (tetrisBlocksInRegion.empty())
                continue;

            uint64_t oob = ~board;
            if (hasNegations)
                oob = 0;

            if (!RecursivelyPlacePieces(0, board, oob, 0, 0))
                return false;
        }
    }
    return true;
}

template<int width, int height>
bool Witness<width, height>::GoalTest(const WitnessState<width, height> &node) const
{
    // TODO: make this more efficient
    for (int x = 0; x < width + 1; x++)
    {
        for (int y = 0; y < height + 1; y++)
        {
            if (GetMustCrossConstraint(x, y) && (!node.Occupied(x, y))) return false;
            if (GetCannotCrossConstraint(x, y) && (node.Occupied(x, y))) return false;
        }
    }
    for (int x = 0; x < width; x++)
    {
        for (int y = 0; y <= height; y++)
        {
            if (GetMustCrossConstraint(true, x, y) && !node.OccupiedEdge(x, y, x + 1, y)) return false;
            if (GetCannotCrossConstraint(true, x, y) && node.OccupiedEdge(x, y, x + 1, y)) return false;
        }
    }
    for (int x = 0; x <= width; x++)
    {
        for (int y = 0; y < height; y++)
        {
            if (GetMustCrossConstraint(false, x, y) && !node.OccupiedEdge(x, y, x, y + 1)) return false;
            if (GetCannotCrossConstraint(false, x, y) && node.OccupiedEdge(x, y, x, y + 1)) return false;
        }
    }
    //	for (auto &c : mustCrossConstraints)
    //	{
    //		if (!node.Occupied(c.first, c.second))
    //			return false;
    //	}
    //	// First pass - mustCross
    //	for (auto &c : mustCrossEdgeConstraints)
    //	{
    ////		printf("Checking (%d, %d) to (%d, %d) - ", c.location.first, c.location.second,
    /// c.location.first+(c.horiz?1:0), c.location.second+(c.horiz?0:1));
    //		if (!node.OccupiedEdge(c.location.first, c.location.second, c.location.first+(c.horiz?1:0),
    // c.location.second+(c.horiz?0:1)))
    //		{
    ////			printf("Failure\n");
    //			return false;
    //		}
    ////		printf("Success\n");
    //	}
    //
    //	for (auto &c : cannotCrossConstraints)
    //	{
    //		if (node.Occupied(c.first, c.second))
    //			return false;
    //	}
    //	// First pass - mustCross
    //	for (auto &c : cannotCrossEdgeConstraints)
    //	{
    //		//		printf("Checking (%d, %d) to (%d, %d) - ", c.location.first, c.location.second,
    // c.location.first+(c.horiz?1:0), c.location.second+(c.horiz?0:1)); 		if (node.OccupiedEdge(c.location.first,
    // c.location.second, c.location.first+(c.horiz?1:0), c.location.second+(c.horiz?0:1)))
    //		{
    //			//			printf("Failure\n");
    //			return false;
    //		}
    //		//		printf("Success\n");
    //	}

    // Didn't hit end of puzzle
    if (node.path.size() == 0) return false;
    // have to be off
    //	if (!
    //		(node.path.back().second > height || node.path.back().first > width ||
    //		node.path.back().second < 0 || node.path.back().first < 0))
    //		return false;
    if (node.path.back().second <= height && node.path.back().first <= width && node.path.back().second >= 0 &&
        node.path.back().first >= 0)
        return false;

    if (constraintCount[kTriangle] > 0)
        // if (triangleCount > 0)
    {
        for (int x = 0; x < width; x++)
        {
            for (int y = 0; y < height; y++)
            {
                if (regionConstraints[x][y].type == kTriangle) // if (triangleConstraints[y*width+x] > 0)
                {
                    int count = node.OccupiedEdge(x, y, x, y + 1);
                    count += node.OccupiedEdge(x, y, x + 1, y);
                    count += node.OccupiedEdge(x + 1, y, x + 1, y + 1);
                    count += node.OccupiedEdge(x, y + 1, x + 1, y + 1);
                    // if (count != triangleConstraints[y*width+x])
                    if (count != regionConstraints[x][y].parameter) return false;
                }
            }
        }
    }

    if (constraintCount[kSeparation] == 0 && constraintCount[kTetris] == 0 && constraintCount[kStar] == 0 &&
        constraintCount[kEraser] == 0)
        //	if (separationCount == 0 && tetrisCount == 0)
        return true;

    LabelRegions(node);

    // TODO: Verify matched constraints by region.
    // TODO: Count the total number of unmatched constraints for checking eraser constraints and for displaying failed
    // constraints

    if (constraintCount[kSeparation] > 0)
    {
        for (auto &v: regionList) // vector of locations
        {
            bool found = false;
            rgbColor c;
            for (auto &i: *v)
            {
                int x = GetRegionFromX(i); // l%width;
                int y = GetRegionFromY(i); // l/width;

                // if (separationConstraints[i].valid)
                if (regionConstraints[x][y].type == kSeparation)
                {
                    if (!found)
                    {
                        c = regionConstraints[x][y].color; // separationConstraints[i].color;
                        found = true;
                    }
                    else if (c != regionConstraints[x][y].color) // separationConstraints[i].color)
                    {
                        return false;
                    }
                }
            }
        }
    }

    // TODO: After this is working, see if we can merge the tetris constraints into the separation code
    if (constraintCount[kTetris] == 0 && constraintCount[kNegativeTetris] > 0) return false;

    if (constraintCount[kStar] > 0)
    {
        for (auto &v: regionList) // vector of locations
        {
            for (auto &i: *v)
            {
                int x = GetRegionFromX(i); // l%width;
                int y = GetRegionFromY(i); // l/width;
                rgbColor finishedColor(1.0 / 512.0, 1.0 / 512.0, 1.0 / 512.0);
                // if (separationConstraints[i].valid)
                if (regionConstraints[x][y].type == kStar)
                {
                    if (regionConstraints[x][y].color == finishedColor) continue;

                    finishedColor = regionConstraints[x][y].color; // separationConstraints[i].color;
                    int count = 0;
                    for (auto &r: *v)
                    {
                        int xx = GetRegionFromX(r); // l%width;
                        int yy = GetRegionFromY(r); // l/width;

                        if (regionConstraints[xx][yy].type != kNoRegionConstraint &&
                            regionConstraints[x][y].color == regionConstraints[xx][yy].color)
                        {
                            count++;
                            if (count > 2) return false;
                        }
                    }
                    if (count != 2) return false;
                }
            }
        }
    }

    if (constraintCount[kTetris] > 0)
        //	if (tetrisCount > 0)
    {
        tetrisBlockCount.resize(regionList.size());

        // 1. Collect the tetris constraints for each region
        for (int x = 0; x < regionList.size(); x++)
        {
            const auto &v = *regionList[x]; // v is vector of locations in this region
            tetrisBlockCount[x] = 0;
            for (auto l: v) // individual location
            {
                if (regionConstraints[GetRegionFromX(l)][GetRegionFromY(l)].type == kTetris)
                {
                    int whichConstraint = regionConstraints[GetRegionFromX(l)][GetRegionFromY(l)].parameter;
                    int numPieces = tetrisSize[whichConstraint];
                    tetrisBlockCount[x] += numPieces;
                }
                else if (regionConstraints[GetRegionFromX(l)][GetRegionFromY(l)].type == kNegativeTetris)
                {
                    int whichConstraint = regionConstraints[GetRegionFromX(l)][GetRegionFromY(l)].parameter;
                    int numPieces = tetrisSize[whichConstraint];
                    tetrisBlockCount[x] -= numPieces;
                }
            }
            // 2. Make sure the counts of tetris blocks matches the region size
            if (tetrisBlockCount[x] > 0 && tetrisBlockCount[x] != regionList[x]->size())
            {
                // printf("Region %d has %d tetris blocks and size %lu\n", x, tetrisBlockCount[x],
                // regionList[x].size());
                return false;
            }
        }

        // 3. Do the full layout
        for (int x = 0; x < regionList.size(); x++)
        {
            bool hasNegations = false;
            const auto &v = *regionList[x]; // v is vector of locations in this region
            if (v.size() == 0) continue;

            tetrisBlocksInRegion.resize(0);
            // Get bit map of board
            uint64_t board = 0;

            for (auto l: v) // individual location
            {
                uint64_t xx = GetRegionFromX(l); // l%width;
                uint64_t yy = GetRegionFromY(l); // l/width;
                // x and y are offset from bottom left (screen space)
                // need to convert to 8x8 bitmaps space
                board |= ((1ull << (7 - xx)) << ((7 - yy) * 8));

                if (regionConstraints[GetRegionFromX(l)][GetRegionFromY(l)].type == kTetris)
                {
                    int whichConstraint = regionConstraints[xx][yy].parameter; // tetrisConstraints[l];
                    tetrisBlocksInRegion.push_back(whichConstraint);
                }
                if (regionConstraints[GetRegionFromX(l)][GetRegionFromY(l)].type == kNegativeTetris)
                {
                    int whichConstraint = regionConstraints[xx][yy].parameter; // tetrisConstraints[l];
                    tetrisBlocksInRegion.push_back(-whichConstraint);
                    hasNegations = true;
                }

                //				int whichConstraint = constraints[xx][yy].parameter;//tetrisConstraints[l];
                //				if (whichConstraint > 0)
                //				{
                //					tetrisBlocksInRegion.push_back(whichConstraint);
                //				}
                //				else if (whichConstraint < 0)
                //				{ // negative constraints are just another xor
                //					tetrisBlocksInRegion.push_back(whichConstraint);
                //					hasNegations = true;
                //				}
            }

            if (tetrisBlocksInRegion.size() == 0) continue;

            // Get out of bounds map -- places pieces can't go
            uint64_t oob = ~board;
            if (hasNegations) oob = 0;
            //			printf("Region %d\n", x);
            //			DebugPrint(board);
            //			DebugPrint(oob);

            // 4. Now we have all pieces, recursively try to place them
            if (!RecursivelyPlacePieces(0, board, oob, 0, 0))
                return false; // No way to place them
            //			printf("-%d-\n", 0);
            //			DebugPrint(board, 0);
            //			printf("Region %d successful\n", x);
        }
        return true; // didn't find a way to place them
    }

    return true;
}

template<int width, int height>
bool Witness<width, height>::RecursivelyPlacePieces(
        int curr, uint64_t board, uint64_t oob, uint64_t posFootprint, uint64_t negFootprint) const
{
    //	DebugPrint(board, curr*2);
    if (curr == tetrisBlocksInRegion.size()) return (board == 0) && ((negFootprint & posFootprint) == negFootprint);

    bool neg = false;
    int whichBlock = tetrisBlocksInRegion[curr];
    if (whichBlock < 0)
    {
        neg = true;
        whichBlock = -whichBlock;
    }

    // In theory we should just check where pieces are placed, instead of the whole grid.
    // But, this optimization currently requires that the upper/left corner of the piece bitmap always
    // contains a piece, which it doesn't. Adding a reference point offset for each piece type would
    // solve this, but we are first worried about writing correct code
    for (int x = 0; x < width - tetrisWH[whichBlock][0] + 1; x++)
    {
        for (int y = 0; y < height - tetrisWH[whichBlock][1] + 1; y++)
        {
            // printf("%d in %d,%d\n", whichBlock, x, y);
            uint64_t block = ((tetrisBits64[whichBlock] >> x) >> (8 * y));
            board ^= block;
            if ((board & oob) == 0) // piece not out of bounds
            {
                if (RecursivelyPlacePieces(curr + 1, board, oob, neg ? posFootprint : (posFootprint | block),
                                           neg ? (negFootprint | block) : negFootprint))
                {
                    //					printf("-%d-\n", curr+1);
                    //					DebugPrint(board, 0);
                    return true;
                }
            }
            else
            {
                //				DebugPrint(board, curr*2+4);
            }
            board ^= block; //(tetrisBits64[whichBlock]>>x)>>(8*y);
        }
    }
    return false;
}

template<int width, int height>
uint64_t Witness<width, height>::GetMaxHash() const { assert(false); }

template<int width, int height>
uint64_t Witness<width, height>::GetStateHash(const WitnessState<width, height> &node) const
{
    assert(false);
}

template<int width, int height>
void Witness<width, height>::GetStateFromHash(uint64_t parent, WitnessState<width, height> &s) const
{
    assert(false);
}

template<int width, int height>
uint64_t Witness<width, height>::GetActionHash(WitnessAction act) const { return act; }

template<int width, int height>
constexpr int Witness<width, height>::GetNumPathConstraints()
{
    return width * (height + 1) + (width + 1) * height + (width + 1) * (height + 1);
}

#pragma mark -
#pragma mark Path Constraints
#pragma mark -

template<int width, int height>
void Witness<width, height>::SetMustCrossConstraint(int which)
{
    pathConstraints[which] = kMustCross;
}

template<int width, int height>
bool Witness<width, height>::GetMustCrossConstraint(int which) const
{
    return (pathConstraints[which] == kMustCross);
    //	if (which < width*(height+1))
    //	{
    //		int x = which%(width);
    //		int y = which/(width);
    //		mustCrossEdgeConstraint e = {true, {x, y}};
    //		for (const auto &c : mustCrossEdgeConstraints)
    //			if (e == c)
    //				return true;
    //		return false;
    //	}
    //	which -= width*(height+1);
    //	if (which < (width+1)*height)
    //	{
    //		int x = which%(width+1);
    //		int y = which/(width+1);
    //		mustCrossEdgeConstraint e = {false, {x, y}};
    //		for (const auto &c : mustCrossEdgeConstraints)
    //			if (e == c)
    //				return true;
    //		return false;
    //	}
    //	which -= (width+1)*height;
    //	// constraint on corner
    //	int x = which%(width+1);
    //	int y = which/(width+1);
    //	std::pair<int, int> e = {x, y};
    //	for (const auto &c : mustCrossConstraints)
    //		if (e == c)
    //			return true;
    //	return false;
}

template<int width, int height>
void Witness<width, height>::ClearMustCrossConstraint(int which)
{
    pathConstraints[which] = kNoPathConstraint;
}

template<int width, int height>
bool Witness<width, height>::GetMustCrossConstraint(
        bool horiz, int x, int y) const // { mustCrossEdgeConstraints.push_back({horiz, {x, y}});}
{
    if (horiz)
        return GetMustCrossConstraint(y * width + x);
    else
        return GetMustCrossConstraint(width * (height + 1) + x * height + y);
}

template<int width, int height>
bool Witness<width, height>::GetMustCrossConstraint(
        int x, int y) const // { mustCrossEdgeConstraints.push_back({horiz, {x, y}});}
{
    return GetMustCrossConstraint(width * (height + 1) + (width + 1) * height + (width + 1) * y + x);
}

template<int width, int height>
void Witness<width, height>::AddMustCrossConstraint(bool horiz, int x, int y) // { mustCrossEdgeConstraints.push_back({horiz, {x, y}});}
{
    if (horiz)
        SetMustCrossConstraint(y * width + x);
    else
        SetMustCrossConstraint(width * (height + 1) + x * height + y);
}

template<int width, int height>
void Witness<width, height>::AddMustCrossConstraint(int x, int y) // { mustCrossConstraints.push_back({x, y});}
{
    //	if (which < width*(height+1))
    //	{
    //		int x = which%(width);
    //		int y = which/(width);
    //		AddMustCrossConstraint(true, x, y);
    //		return;
    //	}
    //		which -= width*(height+1);
    //	if (which < (width+1)*height)
    //	{
    //		int x = which%(width+1);
    //		int y = which/(width+1);
    //		AddMustCrossConstraint(false, x, y);
    //		return;
    //	}
    //		which -= (width+1)*height;
    //		// constraint on corner
    //		int x = which%(width+1);
    //		int y = which/(width+1);
    SetMustCrossConstraint(width * (height + 1) + (width + 1) * height + (width + 1) * y + x);
    //		AddMustCrossConstraint(x, y);
}

template<int width, int height>
void Witness<width, height>::AddMustCrossConstraint(int which) // { mustCrossConstraints.push_back({x, y});}
{
    pathConstraints[which] = kMustCross;
}

template<int width, int height>
void Witness<width, height>::RemoveMustCrossConstraint(bool horiz, int x, int y) // { mustCrossEdgeConstraints.pop_back();}
{
    if (horiz)
        RemoveMustCrossConstraint(y * width + x);
    else
        RemoveMustCrossConstraint(width * (height + 1) + x * height + y);

    //	if (which < width*(height+1))
    //	{
    //		int x = which%(width);
    //		int y = which/(width);
    //		RemoveMustCrossConstraint(true, x, y);
    //		return;
    //	}
    //	which -= width*(height+1);
    //	if (which < (width+1)*height)
    //	{
    //		int x = which%(width+1);
    //		int y = which/(width+1);
    //		RemoveMustCrossConstraint(false, x, y);
    //		return;
    //	}
    //	which -= (width+1)*height;
    //	// constraint on corner
    //	int x = which%(width+1);
    //	int y = which/(width+1);
    //	RemoveMustCrossConstraint(x, y);
}

template<int width, int height>
void Witness<width, height>::RemoveMustCrossConstraint(int x, int y) // { mustCrossConstraints.pop_back();}
{
    ClearMustCrossConstraint(width * (height + 1) + (width + 1) * height + (width + 1) * y + x);
}

template<int width, int height>
constexpr int Witness<width, height>::GetNumCannotCrossConstraints() const
{
    return width * (height + 1) + (width + 1) * height + (width + 1) * (height + 1);
}

template<int width, int height>
void Witness<width, height>::SetCannotCrossConstraint(int which)
{
    pathConstraints[which] = kCannotCross;
    //	if (which < width*(height+1))
    //	{
    //		int x = which%(width);
    //		int y = which/(width);
    //		AddCannotCrossConstraint(true, x, y);
    //		return;
    //	}
    //	which -= width*(height+1);
    //	if (which < (width+1)*height)
    //	{
    //		int x = which%(width+1);
    //		int y = which/(width+1);
    //		AddCannotCrossConstraint(false, x, y);
    //		return;
    //	}
    //	which -= (width+1)*height;
    //	// constraint on corner
    //	int x = which%(width+1);
    //	int y = which/(width+1);
    //	AddCannotCrossConstraint(x, y);
}

template<int width, int height>
bool Witness<width, height>::GetCannotCrossConstraint(int which) const
{
    return (pathConstraints[which] == kCannotCross);
    //	if (which < width*(height+1))
    //	{
    //		int x = which%(width);
    //		int y = which/(width);
    //		mustCrossEdgeConstraint e = {true, {x, y}};
    //		for (const auto &c : cannotCrossEdgeConstraints)
    //			if (e == c)
    //				return true;
    //		return false;
    //	}
    //	which -= width*(height+1);
    //	if (which < (width+1)*height)
    //	{
    //		int x = which%(width+1);
    //		int y = which/(width+1);
    //		mustCrossEdgeConstraint e = {false, {x, y}};
    //		for (const auto &c : cannotCrossEdgeConstraints)
    //			if (e == c)
    //				return true;
    //		return false;
    //	}
    //	which -= (width+1)*height;
    //	// constraint on corner
    //	int x = which%(width+1);
    //	int y = which/(width+1);
    //	std::pair<int, int> e = {x, y};
    //	for (const auto &c : cannotCrossConstraints)
    //		if (e == c)
    //			return true;
    //	return false;
}

template<int width, int height>
void Witness<width, height>::ClearCannotCrossConstraint(int which)
{
    pathConstraints[which] = kNoPathConstraint;
    //	if (which < width*(height+1))
    //	{
    //		int x = which%(width);
    //		int y = which/(width);
    //		RemoveCannotCrossConstraint(true, x, y);
    //		return;
    //	}
    //	which -= width*(height+1);
    //	if (which < (width+1)*height)
    //	{
    //		int x = which%(width+1);
    //		int y = which/(width+1);
    //		RemoveCannotCrossConstraint(false, x, y);
    //		return;
    //	}
    //	which -= (width+1)*height;
    //	// constraint on corner
    //	int x = which%(width+1);
    //	int y = which/(width+1);
    //	RemoveCannotCrossConstraint(x, y);
}

template<int width, int height>
bool Witness<width, height>::GetCannotCrossConstraint(
        int x, int y) const // { mustCrossEdgeConstraints.push_back({horiz, {x, y}});}
{
    return GetCannotCrossConstraint(width * (height + 1) + (width + 1) * height + (width + 1) * y + x);
}

template<int width, int height>
bool Witness<width, height>::GetCannotCrossConstraint(
        bool horiz, int x, int y) const // { mustCrossEdgeConstraints.push_back({horiz, {x, y}});}
{
    if (horiz)
        return GetCannotCrossConstraint(y * width + x);
    else
        return GetCannotCrossConstraint(width * (height + 1) + x * height + y);
}

template<int width, int height>
void Witness<width, height>::AddCannotCrossConstraint(
        bool horiz, int x, int y) // { cannotCrossEdgeConstraints.push_back({horiz, {x, y}});}
{
    if (horiz)
        AddCannotCrossConstraint(y * width + x);
    else
        AddCannotCrossConstraint(width * (height + 1) + x * height + y);
}

template<int width, int height>
void Witness<width, height>::AddCannotCrossConstraint(int x, int y) // { cannotCrossConstraints.push_back({x, y});}
{
    SetCannotCrossConstraint(width * (height + 1) + (width + 1) * height + (width + 1) * y + x);
}

template<int width, int height>
void Witness<width, height>::AddCannotCrossConstraint(int which) // { cannotCrossConstraints.push_back({x, y});}
{
    pathConstraints[which] = kCannotCross;
}

template<int width, int height>
void Witness<width, height>::RemoveCannotCrossConstraint(bool horiz, int x,
                                                         int y) // { cannotCrossEdgeConstraints.pop_back();}
{
    if (horiz)
        RemoveCannotCrossConstraint(y * width + x);
    else
        RemoveCannotCrossConstraint(width * (height + 1) + x * height + y);
}

template<int width, int height>
void Witness<width, height>::RemoveCannotCrossConstraint(int x, int y) // { cannotCrossConstraints.pop_back();}
{
    RemoveCannotCrossConstraint(width * (height + 1) + (width + 1) * height + (width + 1) * y + x);
}

#pragma mark -
#pragma mark Solver Helper Functions
#pragma mark -

template<int width, int height>
void Witness<width, height>::LabelRegions(const WitnessState<width, height> &s) const
{
    regions.fill(0);
    //	regions.clear();
    //	regions.resize(width*height);
    for (std::vector<int> *i: regionList)
    {
        // printf("Returned %p; ", i);
        regionCache.returnItem(i);
    }
    regionList.resize(0);
    // printf("Size to %lu\n", regionList.size());

    // static
    std::vector<int> queue;
    queue.clear();

    int index = 0;
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            if (regions[GetRegionIndex(x, y) /*y*width+x*/] == 0)
            {
                queue.push_back(GetRegionIndex(x, y));
                index++;
                while (regionList.size() < index)
                {
                    regionList.push_back(regionCache.getItem());
                    //					printf("Got %p; ", regionList.back());
                    //					printf("size to %lu\n", regionList.size());
                }
                // regionList.resize(index);
            }
            while (queue.size() > 0)
            {
                int next = queue.back();
                queue.pop_back();

                if (regions[next] == 0)
                {
                    regions[next] = index;
                    regionList[index - 1]->push_back(next);
                    // check 4 directions
                    int xx = GetRegionFromX(next);
                    int yy = GetRegionFromY(next);
                    if (xx > 0 && !s.OccupiedEdge(xx, yy, xx, yy + 1) && regions[GetRegionIndex(xx - 1, yy)] == 0)
                        queue.push_back(GetRegionIndex(xx - 1, yy));
                    if (xx < width - 1 && !s.OccupiedEdge(xx + 1, yy, xx + 1, yy + 1) &&
                        regions[GetRegionIndex(xx + 1, yy)] == 0)
                        queue.push_back(GetRegionIndex(xx + 1, yy));
                    if (yy > 0 && !s.OccupiedEdge(xx, yy, xx + 1, yy) && regions[GetRegionIndex(xx, yy - 1)] == 0)
                        queue.push_back(GetRegionIndex(xx, yy - 1));
                    if (yy < height - 1 && !s.OccupiedEdge(xx, yy + 1, xx + 1, yy + 1) &&
                        regions[GetRegionIndex(xx, yy + 1)] == 0)
                        queue.push_back(GetRegionIndex(xx, yy + 1));
                }
            }
        }
    }

    //	for (auto i : s.path)
    //		std::cout << "(" << i.first << ", " << i.second << ") ";
    //	std::cout << "\n";
    //
    //	printf(" ");
    //	for (int x = 0; x < width; x++)
    //	{
    //		if (s.OccupiedEdge(x, 0, x+1, 0))
    //			printf("- ");
    //		else
    //			printf("  ");
    //	}
    //	printf("\n");
    //
    //	for (int y = 0; y < height; y++)
    //	{
    //		if (s.OccupiedEdge(0, y, 0, y+1))
    //			printf("|");
    //		else
    //			printf(" ");
    //		for (int x = 0; x < width; x++)
    //		{
    //			printf("%d", regions[GetIndex(x, y)]);
    //			if (s.OccupiedEdge(x+1, y, x+1, y+1))
    //				printf("|");
    //			else
    //				printf(" ");
    //		}
    //		printf("\n");
    //		printf(" ");
    //		for (int x = 0; x < width; x++)
    //		{
    //			if (s.OccupiedEdge(x, y+1, x+1, y+1))
    //				printf("- ");
    //			else
    //				printf("  ");
    //		}
    //		printf("\n");
    //	}
    //	printf("\n\n");
}

#pragma mark -
#pragma mark GUI
#pragma mark -

template<int width, int height>
bool Witness<width, height>::Click(Graphics::point mouseLoc, InteractiveWitnessState<width, height> &iws)
{
    // in start area
    // std::cout << mouseLoc-GetScreenCoord(0, 0) << " size " << (mouseLoc-GetScreenCoord(0, 0)).length() << " vs " <<
    // 3*lineWidth << "\n";
    if ((iws.currState == InteractiveWitnessState<width, height>::kWaitingStart ||
         iws.currState == InteractiveWitnessState<width, height>::kWaitingRestart) &&
        (mouseLoc - GetScreenCoord(start[0].first, start[0].second)).length() < 3 * lineWidth)
    {
        // printf("Switched from watiting to start to kInPoint\n");
        ApplyAction(iws.ws, kStart);
        iws.currState = InteractiveWitnessState<width, height>::kInPoint;
        return false;
    }

    if (iws.ws.path.size() > 0 &&
        (iws.ws.path.back().second > height || iws.ws.path.back().second < 0 || iws.ws.path.back().first > width ||
         iws.ws.path.back().first < 0)) // at goal, may not be solution
    {
        iws.currState = InteractiveWitnessState<width, height>::kWaitingRestart;
        return true;
    }

    if (iws.target.first < 0 || iws.target.first > width || iws.target.second < 0 || iws.target.second > height)
    {
        // mouse is off screen; might be trying to exit with mouse out of the goal
        if (Legal(iws.ws, kEnd))
        {
            ApplyAction(iws.ws, kEnd);
            iws.currState = InteractiveWitnessState<width, height>::kWaitingRestart;
            return true;
        }
    }

    // quit path
    iws.Reset();
    return false;
}

template<int width, int height>
void Witness<width, height>::Move(Graphics::point mouseLoc, InteractiveWitnessState<width, height> &iws)
{
    if (iws.currState == InteractiveWitnessState<width, height>::kWaitingStart ||
        iws.currState == InteractiveWitnessState<width, height>::kWaitingRestart)
    {
        return;
    }

    if (iws.currState == InteractiveWitnessState<width, height>::kInPoint)
    {
        Graphics::point end = GetScreenCoord(iws.ws.path.back().first, iws.ws.path.back().second);

        float factor = 3;
        if (iws.ws.path.size() > 1) factor = 1;

        // check if we left the point
        if (!Graphics::PointInRect(mouseLoc, Graphics::rect(end, factor * lineWidth)))
        {
            // 1. Find closest successor to mouse
            // find where to add to path
            std::vector <WitnessAction> moves;
            GetMouseActions(iws.ws, moves);

            iws.target = iws.ws.path.back();
            WitnessAction a = moves[0];
            float dist = 1000;
            for (auto m: moves)
            {
                iws.target = iws.ws.path.back();
                ApplyAction(iws.target, m);
                Graphics::point p = GetScreenCoord(iws.target.first, iws.target.second);
                // std::cout << "Dist from " << mouseLoc << " to " << p << " is " << ((mouseLoc-p).length()) << " act: "
                // << m << "\n";
                if ((mouseLoc - p).length() < dist)
                {
                    dist = (mouseLoc - p).length();
                    a = m;
                }
            }

            // 2. Add to target location
            iws.target = iws.ws.path.back();
            ApplyAction(iws.target, a);
            iws.targetAct = a;

            // 3. Change state
            // printf("Switched from kInPoint to kBetweenPoints\n");
            iws.currState = InteractiveWitnessState<width, height>::kBetweenPoints;
            iws.frac = 0;

            int len = static_cast<int>(iws.ws.path.size());

            if (len >= 2 && iws.target == iws.ws.path[len - 2]) // going backwards
            {
                iws.target = iws.ws.path.back();
                InvertAction(iws.targetAct);
                iws.frac = 1.0;
                UndoAction(iws.ws, a);
                // printf("Switched from kInPoint to kBetweenPoints [backwards]\n");
                iws.currState = InteractiveWitnessState<width, height>::kBetweenPoints;
            }
        }

        return;
    }

    if (iws.currState == InteractiveWitnessState<width, height>::kBetweenPoints)
    {
        Graphics::point from = GetScreenCoord(iws.ws.path.back().first, iws.ws.path.back().second);
        Graphics::point to = GetScreenCoord(iws.target.first, iws.target.second);

        // check if we entered the end (legally)
        if (Graphics::PointInRect(mouseLoc, Graphics::rect(to, lineWidth)) && Legal(iws.ws, iws.targetAct))
        {
            iws.currState = InteractiveWitnessState<width, height>::kInPoint;
            ApplyAction(iws.ws, iws.targetAct);
            //			printf("Switched from kBetweenPoints to kInPoint [1]\n");
        }
        else if (from.x == to.x && (mouseLoc.y - from.y) / (to.y - from.y) > 0.9 &&
                 Legal(iws.ws, iws.targetAct)) // tracking in y
        {
            iws.currState = InteractiveWitnessState<width, height>::kInPoint;
            ApplyAction(iws.ws, iws.targetAct);
            //			printf("Switched from kBetweenPoints to kInPoint [2]\n");
        }
        else if (from.y == to.y && (mouseLoc.x - from.x) / (to.x - from.x) > 0.9 &&
                 Legal(iws.ws, iws.targetAct)) // tracking in x
        {
            iws.currState = InteractiveWitnessState<width, height>::kInPoint;
            ApplyAction(iws.ws, iws.targetAct);
            //			printf("Switched from kBetweenPoints to kInPoint [3]\n");
        }
            // entered start
        else if (Graphics::PointInRect(mouseLoc, Graphics::rect(from, lineWidth)))
        {
            iws.currState = InteractiveWitnessState<width, height>::kInPoint;
            //			printf("Switched from kBetweenPoints to kInPoint [backtrack:1]\n");
        }
        else if (from.x == to.x && (mouseLoc.y - from.y) / (to.y - from.y) < 0.1) // tracking in y
        {
            iws.currState = InteractiveWitnessState<width, height>::kInPoint;
            //			printf("Switched from kBetweenPoints to kInPoint [backtrack:2]\n");
        }
        else if (from.y == to.y && (mouseLoc.x - from.x) / (to.x - from.x) < 0.1) // tracking in x
        {
            iws.currState = InteractiveWitnessState<width, height>::kInPoint;
            //			printf("Switched from kBetweenPoints to kInPoint [backtrack:3]\n");
        }

        // If still tracking, update track distance
        if (iws.currState == InteractiveWitnessState<width, height>::kBetweenPoints)
        {
            double gapSize;
            if (from.x == to.x) // tracking in y
            {
                iws.frac = (mouseLoc.y - from.y) / (to.y - from.y);
                gapSize = lineWidth / fabs(to.y - from.y);
            }
            else
            {
                iws.frac = (mouseLoc.x - from.x) / (to.x - from.x);
                gapSize = lineWidth / fabs(to.x - from.x);
            }

            if (iws.frac > 1) iws.frac = 1;
            legality l;
            if (!Legal(iws.ws, iws.targetAct, l))
            {
                // hitting start circle
                if ((iws.target == std::pair<int, int>(0, 0)) && iws.frac > 0.6f)
                    iws.frac = 0.6f;
                else if (iws.frac > 0.5 - 2 * gapSize && l == kHitCannotCross) // hitting cannot cross constraint
                    iws.frac = 0.5 - 2 * gapSize;
                else if (iws.frac > 0.8f && l == kHitLine) // hitting line
                    iws.frac = 1.f - 2 * gapSize;          // 0.8f;
            }
            //				if (!Legal(iws.ws, iws.targetAct) && iws.frac > 0.8)
            //					iws.frac = 0.8;
            if (iws.frac < 0) iws.frac = 0;
        }
    }
}

#pragma mark -
#pragma mark Visualization
#pragma mark -

template<int width, int height>
void Witness<width, height>::DrawRegionConstraint(
        Graphics::Display &display, const WitnessRegionConstraint &constraint, const Graphics::point &p3) const
{
    switch (constraint.type)
    {
    case kNoRegionConstraint:
        break;
    case kSeparation:
    {
        Graphics::point delta = Graphics::point{lineWidth, lineWidth};
        display.FillCircle(p3 + delta, lineWidth, constraint.color);
        display.FillCircle(p3 - delta, lineWidth, constraint.color);
        delta.x = -delta.x;
        display.FillCircle(p3 + delta, lineWidth, constraint.color);
        display.FillCircle(p3 - delta, lineWidth, constraint.color);
        display.FillRect(
                {p3.x - 1.0f * lineWidth, p3.y - 2.0f * lineWidth, p3.x + 1.0f * lineWidth,
                 p3.y + 2.0f * lineWidth},
                constraint.color);
        display.FillRect(
                {p3.x - 2.0f * lineWidth, p3.y - 1.0f * lineWidth, p3.x + 2.0f * lineWidth,
                 p3.y + 1.0f * lineWidth},
                constraint.color);
        break;
    }
    case kStar:
    {
        display.FillNGon(p3, gapOffset / 5.0f, 4, 0, constraint.color);
        display.FillNGon(p3, gapOffset / 5.0f, 4, 45, constraint.color);
        break;
    }
    case kTetris:
    case kNegativeTetris:
    {
        int whichPiece = abs(constraint.parameter);
        bool negative = constraint.type == kNegativeTetris; // constraint.parameter<0;
        if (whichPiece != 0)
        {
            float xOff = 0;
            float yOff = 0;
            switch (whichPiece)
            {
            case 1:
                xOff = 1.5;
                yOff = 1.5;
                break;
            case 2:
                xOff = 1;
                yOff = 1.5;
                break;
            case 3:
                xOff = 1.5;
                yOff = 1;
                break;
            case 4:
            case 5:
            case 6:
            case 7:
            case 8:
                xOff = 0.5;
                yOff = 1.5;
                break;
            case 9:
                xOff = 1.5;
                yOff = 0.5;
                break;
            case 10:
                xOff = 1;
                yOff = 1;
                break;
            case 11:
            case 12:
            case 13:
            case 14:
                xOff = 0.5;
                yOff = 1;
                break;
            case 15:
            case 16:
            case 17:
            case 18:
                xOff = 1;
                yOff = 0.5;
                break;
            case 19:
                xOff = 0;
                yOff = 1.5;
                break;
            case 20:
                xOff = 1.5;
                yOff = 0;
                break;
            case 21:
            case 22:
                xOff = 0.5;
                yOff = 1;
                break;
            case 23:
            case 24:
                xOff = 1.0;
                yOff = 0.5;
                break;
            }
            float blockSize = gapOffset / 8.0f;
            for (int yy = 0; yy < 4; yy++)
            {
                for (int xx = 0; xx < 4; xx++)
                {
                    if ((tetrisBits[whichPiece] >> (yy * 4 + xx)) & 1)
                    {
                        Graphics::point p4(-2 * blockSize + xx * blockSize + 0.5f * blockSize - xOff * blockSize,
                                           -2 * blockSize + yy * blockSize + 0.5f * blockSize - yOff * blockSize);
                        Graphics::rect r((p3 - p4), blockSize * 0.35f);
                        if (negative)
                        {
                            display.FillRect(r, tetrisBlue);
                            Graphics::rect inner((p3 - p4), blockSize * 0.35f * 0.55f);
                            display.FillRect(inner, backColor);
                        }
                        else
                            display.FillRect(r, tetrisYellow);
                    }
                }
            }
        }
        break;
    }
    case kTriangle:
    {
        switch (constraint.parameter)
        {
        case 1:
        {
            display.FillNGon(p3, lineWidth * 0.9f, 3, 60, Colors::orange);
            break;
        }
        case 2:
        {
            Graphics::point p = p3;
            p.x -= lineWidth;
            display.FillNGon(p, lineWidth * 0.9f, 3, 60, Colors::orange);
            p.x += 2 * lineWidth;
            display.FillNGon(p, lineWidth * 0.9f, 3, 60, Colors::orange);
            break;
        }
        case 3:
        {
            Graphics::point p = p3;
            display.FillNGon(p, lineWidth * 0.9f, 3, 60, Colors::orange);
            p.x -= 2 * lineWidth;
            display.FillNGon(p, lineWidth * 0.9f, 3, 60, Colors::orange);
            p.x += 4 * lineWidth;
            display.FillNGon(p, lineWidth * 0.9f, 3, 60, Colors::orange);
            break;
        }
        }
        break;
    }
    case kEraser:
        break;
    case kRegionConstraintCount:
        break;
    }
}

template<int width, int height>
void Witness<width, height>::Draw(Graphics::Display &display) const
{
    // Background drawing
    {
        Graphics::point p1 = GetScreenCoord(0, 0);
        Graphics::point p2 = GetScreenCoord(width, height);

        display.FillRect({-1, -1, 1, 1}, outerBackColor);
        display.FillRect({p1.x, p2.y, p2.x, p1.y}, backColor);
    }

    for (int x = 0; x <= width; x++)
    {
        DoLine(display, GetScreenCoord(x, 0), GetScreenCoord(x, height), lineColor);
        // display.FillRect({(-1+gapOffset*x-lineWidth)*scale, -1*scale, (-1+gapOffset*x+lineWidth)*scale, 1*scale},
        // Colors::lightgray);
    }
    for (int y = 0; y <= height; y++)
    {
        DoLine(display, GetScreenCoord(0, y), GetScreenCoord(width, y), lineColor);
        // display.FillRect({-1*scale, (-1+gapOffset*y-lineWidth)*scale, 1*scale, (-1+gapOffset*y+lineWidth)*scale},
        // Colors::lightgray);
    }

    // corners -- all in case we move the start/goal
    display.FillCircle(GetScreenCoord(0, height), lineWidth, lineColor);
    display.FillCircle(GetScreenCoord(width, 0), lineWidth, lineColor);
    display.FillCircle(GetScreenCoord(0, 0), lineWidth, lineColor);
    display.FillCircle(GetScreenCoord(width, height), lineWidth, lineColor);

    // start
    assert(start.size() > 0);
    for (const auto &i: start)
        display.FillCircle(GetScreenCoord(i.first, i.second), lineWidth * 3.f, lineColor);

    // Draw end points. Would be more efficient to iterate goals, but then we have to
    // remap them onto edges. Loss of efficiency is negligible
    for (int x = 0; x <= width; x++)
    {
        for (int y = 0; y <= width; y++)
        {
            int val;
            if ((val = goalMap[GetPathIndex(x, y)]) != 0)
            {
                auto endLoc = GetScreenCoord(goal[val - 1].first, goal[val - 1].second);
                DoLine(display, GetScreenCoord(x, y), endLoc, lineColor);
                display.FillCircle(endLoc, lineWidth, lineColor);
            }
        }
    }
    //	for (const auto &i : goal)
    //	{
    //		auto endLoc = GetScreenCoord((i.first == width)?(width+1):(i.first==0?-1:i.first),
    //									(i.second==height)?height+1:(i.second==0?-1:i.second));
    //		auto gridLoc = GetScreenCoord(i.first, i.second);
    //		DoLine(display, gridLoc, endLoc, lineColor);
    //		display.FillCircle(endLoc, lineWidth, lineColor);
    //	}

    for (int x = 0; x < width + 1; x++)
    {
        for (int y = 0; y < height + 1; y++)
        {
            if (GetMustCrossConstraint(x, y))
            {
                Graphics::point pt = GetScreenCoord(x, y);
                display.FillNGon(pt, lineWidth * 0.9f, 6, 30, Colors::black);
            }
            else if (GetCannotCrossConstraint(x, y))
            {
                Graphics::point pt = GetScreenCoord(x, y);
                Graphics::rect r(pt, lineWidth);
                display.FillSquare(pt, lineWidth, backColor);
                if (x > 0)
                {
                    Graphics::point pt2 = GetScreenCoord(x - 1, y);
                    //			pt2 = (pt+pt2)*0.5;
                    pt2.x += lineWidth;
                    Graphics::rect r2(pt2, 0);
                    r2 |= r;
                    display.FillRect(r2, backColor);
                }
                if (x + 1 <= width)
                {
                    Graphics::point pt2 = GetScreenCoord(x + 1, y);
                    // pt2 = (pt+pt2)*0.5;
                    pt2.x -= lineWidth;
                    Graphics::rect r2(pt2, 0);
                    r2 |= r;
                    display.FillRect(r2, backColor);
                }
                if (y > 0)
                {
                    Graphics::point pt2 = GetScreenCoord(x, y - 1);
                    // pt2 = (pt+pt2)*0.5;
                    pt2.y -= lineWidth;
                    Graphics::rect r2(pt2, 0);
                    r2 |= r;
                    display.FillRect(r2, backColor);
                }
                if (y + 1 <= width)
                {
                    Graphics::point pt2 = GetScreenCoord(x, y + 1);
                    // pt2 = (pt+pt2)*0.5;
                    pt2.y += lineWidth;

                    Graphics::rect r2(pt2, 0);
                    r2 |= r;
                    display.FillRect(r2, backColor);
                }
            }
        }
    }
    for (int x = 0; x < width; x++)
    {
        for (int y = 0; y <= height; y++)
        {
            if (GetMustCrossConstraint(true, x, y))
            {
                Graphics::point p1 = GetScreenCoord(x, y);
                Graphics::point p2 = GetScreenCoord(x + 1, y);
                Graphics::point pt = (p1 + p2) * 0.5;
                display.FillNGon(pt, lineWidth * 0.9f, 6, 30, Colors::black);
            }
            else if (GetCannotCrossConstraint(true, x, y))
            {
                Graphics::point p1 = GetScreenCoord(x, y);
                Graphics::point p2 = GetScreenCoord(x + 1, y);
                Graphics::point pt = (p1 + p2) * 0.5;
                display.FillSquare(pt, lineWidth, backColor);
            }
        }
    }
    for (int x = 0; x <= width; x++)
    {
        for (int y = 0; y < height; y++)
        {
            if (GetMustCrossConstraint(false, x, y))
            {
                Graphics::point p1 = GetScreenCoord(x, y);
                Graphics::point p2 = GetScreenCoord(x, y + 1);
                Graphics::point pt = (p1 + p2) * 0.5;
                display.FillNGon(pt, lineWidth * 0.9f, 6, 30, Colors::black);
            }
            else if (GetCannotCrossConstraint(false, x, y))
            {
                Graphics::point p1 = GetScreenCoord(x, y);
                Graphics::point p2 = GetScreenCoord(x, y + 1);
                Graphics::point pt = (p1 + p2) * 0.5;
                display.FillSquare(pt, lineWidth, backColor);
            }
        }
    }

    //	// must-cross constraints (polygons)
    //	for (auto &c : mustCrossEdgeConstraints)
    //	{
    //		Graphics::point p1 = GetScreenCoord(c.location.first, c.location.second);
    //		Graphics::point p2 = GetScreenCoord(c.location.first+(c.horiz?1:0), c.location.second+(c.horiz?0:1));
    //		Graphics::point pt = (p1+p2)*0.5;
    //		display.FillNGon(pt,  lineWidth*0.9f, 6, 30, Colors::black);
    //	}
    //	for (auto &c : mustCrossConstraints)
    //	{
    //		Graphics::point pt = GetScreenCoord(c.first, c.second);
    //		display.FillNGon(pt,  lineWidth*0.9f, 6, 30, Colors::black);
    //	}
    //
    //	// must-cross constraints (polygons)
    //	for (auto &c : cannotCrossEdgeConstraints)
    //	{
    //		Graphics::point p1 = GetScreenCoord(c.location.first, c.location.second);
    //		Graphics::point p2 = GetScreenCoord(c.location.first+(c.horiz?1:0), c.location.second+(c.horiz?0:1));
    //		Graphics::point pt = (p1+p2)*0.5;
    //		display.FillSquare(pt, lineWidth, backColor);
    //	}
    //	for (auto &c : cannotCrossConstraints)
    //	{
    //		Graphics::point pt = GetScreenCoord(c.first, c.second);
    //		Graphics::rect r(pt, lineWidth);
    //		display.FillSquare(pt, lineWidth, backColor);
    //		if (c.first > 0)
    //		{
    //			Graphics::point pt2 = GetScreenCoord(c.first-1, c.second);
    ////			pt2 = (pt+pt2)*0.5;
    //			pt2.x += lineWidth;
    //			Graphics::rect r2(pt2, 0);
    //			r2 |= r;
    //			display.FillRect(r2, backColor);
    //		}
    //		if (c.first+1 <= width)
    //		{
    //			Graphics::point pt2 = GetScreenCoord(c.first+1, c.second);
    //			//pt2 = (pt+pt2)*0.5;
    //			pt2.x -= lineWidth;
    //			Graphics::rect r2(pt2, 0);
    //			r2 |= r;
    //			display.FillRect(r2, backColor);
    //		}
    //		if (c.second > 0)
    //		{
    //			Graphics::point pt2 = GetScreenCoord(c.first, c.second-1);
    //			//pt2 = (pt+pt2)*0.5;
    //			pt2.y -= lineWidth;
    //			Graphics::rect r2(pt2, 0);
    //			r2 |= r;
    //			display.FillRect(r2, backColor);
    //		}
    //		if (c.second+1 <= width)
    //		{
    //			Graphics::point pt2 = GetScreenCoord(c.first, c.second+1);
    //			//pt2 = (pt+pt2)*0.5;
    //			pt2.y += lineWidth;
    //
    //			Graphics::rect r2(pt2, 0);
    //			r2 |= r;
    //			display.FillRect(r2, backColor);
    //		}
    //	}

    for (int x = 0; x < width; x++)
    {
        for (int y = 0; y < height; y++)
        {
            Graphics::point p1 = GetScreenCoord(x, y);
            Graphics::point p2 = GetScreenCoord(x + 1, y + 1);
            Graphics::point pt = (p1 + p2) * 0.5;
            DrawRegionConstraint(display, regionConstraints[x][y], pt);
        }
    }
}

template<int width, int height>
void Witness<width, height>::Draw(Graphics::Display &display, const WitnessState<width, height> &s) const
{
    if (s.path.size() == 0)
    {
        return;
    }
    display.FillCircle(GetScreenCoord(start[0].first, start[0].second), lineWidth * 3.f, drawColor);
    for (int x = 1; x < s.path.size(); x++)
    {
        Graphics::point p1 = GetScreenCoord(s.path[x - 1].first, s.path[x - 1].second);
        Graphics::point p2 = GetScreenCoord(s.path[x].first, s.path[x].second);

        DoLine(display, p1, p2, drawColor);

        if (x > 1)
        {
            display.FillCircle(p1, lineWidth, drawColor);
        }
        display.FillCircle(p2, lineWidth, drawColor);
        //		if (x == s.path.size()-1)
        //			display.FillCircle(p2, lineWidth*0.75f, lineColor);
    }
}

template<int width, int height>
void Witness<width, height>::Draw(Graphics::Display &display, const InteractiveWitnessState<width, height> &iws) const
{
    //	float scale = 0.8f;

    // draw animated start marker
    if (iws.currState == InteractiveWitnessState<width, height>::kWaitingStart)
    {
        assert((iws.ws.path.size() == 0));
        if (iws.frac <= 1)
            display.FrameCircle(GetScreenCoord(start[0].first, start[0].second), lineWidth * 3.f * iws.frac, drawColor,
                                lineWidth * 0.5f);
        return;
    }

    // draw main state
    Draw(display, iws.ws);

    if (iws.currState == InteractiveWitnessState<width, height>::kBetweenPoints)
    {
        // draw last fraction
        Graphics::point p1 = GetScreenCoord(iws.ws.path.back().first, iws.ws.path.back().second);
        Graphics::point p2 = GetScreenCoord(iws.target.first, iws.target.second);
        p2 = p1 + (p2 - p1) * iws.frac;
        DoLine(display, p1, p2, drawColor);
        display.FillCircle(p2, lineWidth, drawColor);
        display.FillCircle(p2, lineWidth * 0.5f, lineColor);
    }
    else
    {
        Graphics::point p2 = GetScreenCoord(iws.ws.path.back().first, iws.ws.path.back().second);
        display.FillCircle(p2, lineWidth * 0.5f, lineColor);
    }
}

template<int width, int height>
inline std::ostream& operator<<(std::ostream &os, const Witness<width, height> &witness)
{
    return witness.Serialize(os);
}

template<int width, int height>
inline std::istream& operator>>(std::istream &is, Witness<width, height> &witness)
{
    return witness.Deserialize(is);
}

#endif // HOG2_ENVIRONMENTS_WITNESS_H
