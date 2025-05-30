#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <sstream>
#include <set>
#include <cstdlib>
#include <ctime>
#include <vector>
#include <thread>
#include <chrono>
#include <algorithm>
#include <queue>
#include <random>
#include <unordered_map>

using namespace std;

class Battlefield;

// Class for the robot
class Robot {
protected:
    string id_;
    int robotPositionX, robotPositionY;
    string robotType_;
    int numOfLives_ = 3;
    bool isValid;

public:
    virtual ~Robot() {}
    Robot(string id = "", int x = -1, int y = -1) : id_(id), robotPositionX(x), robotPositionY(y), isValid(true) {}

    //virtual void actionMove() = 0;
    //virtual void actionFire() = 0;
    //virtual void actionLook() = 0;

    // Add virtual actions method
    //virtual void actions() {
    //    actionMove();
    //    actionFire();
    //    actionLook();
    //}

    int x() const { return robotPositionX; }
    void setLocationX(int x) { robotPositionX = x; }

    int y() const { return robotPositionY; }
    void setLocationY(int y) { robotPositionY = y; }

    void setLocation(int x, int y) {
        robotPositionX = x;
        robotPositionY = y;
    }

    string id() const { return id_; }
    void setId(const string& id) { id_ = id; }

    int numOfLives() const { return numOfLives_; }
    void setNumOfLives(int lives) { numOfLives_ = lives; }

    bool exists() const { return isValid; }
    void destroy() { isValid = false; }

    virtual void reEnter(int x, int y) {
        robotPositionX = x;
        robotPositionY = y;
        numOfLives_ = 3;
        isValid = true;
    }
};

class Battlefield {
private:
    int rows, cols;
    vector<vector<string>> grid;  // 2D grid to represent the battlefield
    vector<Robot*> robots;  // A list of active robots
    queue<Robot*> destroyedRobotsQueue;  // Queue for robots waiting to re-enter

public:
    Battlefield(int r, int c) : rows(r), cols(c) {
        grid.resize(rows, vector<string>(cols, ""));
    }

    // Renaming the method to avoid conflict
    const vector<Robot*> getRobots() const { return robots; }


    int BATTLEFIELD_NUM_OF_COLS() const { return cols; }
    int BATTLEFIELD_NUM_OF_ROWS() const { return rows; }

    void addRobot(Robot* robot) {
        robots.push_back(robot);
        grid[robot->y()][robot->x()] = robot->id();
    }

    bool isOccupied(int x, int y) {
        return grid[y][x] != "";
    }

    void updateBattlefield() {
        for (int i = 0; i < rows; ++i){
            for (int j = 0; j < cols; ++j){
                grid[i][j] = "";
            }
        }
        for (auto& robot : robots){
            if(robot->exists()){
                grid[robot->y()][robot->x()] = robot->id();
            }
        }
    }

    void printBattlefield() {
        for (int i = 0; i < rows; ++i){
            for (int j = 0; j < cols; ++j){
                if(grid[i][j] != ""){
                    cout << grid[i][j] << "\t";
                } else {
                    cout << "Empty\t";
                }
            }
            cout << endl;
        }
    }

    void handleDestroyedRobot(Robot* robot) {
        destroyedRobotsQueue.push(robot);
    }

    bool reEnterRobot() {
        if (!destroyedRobotsQueue.empty()){
            Robot* robotToReEnter = destroyedRobotsQueue.front();
            destroyedRobotsQueue.pop();
            if (robotToReEnter->numOfLives() > 0){
                int randX = rand() % cols;
                int randY = rand() % rows;
                robotToReEnter->setLocation(randX, randY);
                robotToReEnter->reEnter(randX, randY);
                addRobot(robotToReEnter);
                cout << robotToReEnter->id() << " has re-entered the battlefield at (" << randX << ", " << randY << ")." << endl;
                return true;
            }
        }
        return false;
    }

    // Added method to erase robot by ID
    void eraseRobotById(const string& id) {
        auto it = std::remove_if(robots.begin(), robots.end(),
            [&](Robot* robot) {
                if (robot->id() == id) {
                    grid[robot->y()][robot->x()] = "";
                    delete robot;  // free memory
                    return true;
                }
                return false;
            }
        );
        robots.erase(it, robots.end());
        updateBattlefield();
    }

    // Provide access to grid for ShootingRobot fire method
    const vector<vector<string>>& getGrid() const {
        return grid;
    }

    // Getters for rows and cols
    int getCols() const { return cols; }
    int getRows() const { return rows; }
};


class MovingRobot: virtual public Robot{
protected:
    int attempts = 0;
    int maxAttempts = 8;
    int tempX, tempY;

public:
    virtual ~MovingRobot(){}
    virtual void actionMove() = 0;
    };


class SeeingRobot: virtual public Robot {
protected:
    int minX = -1, minY = -1, maxX = -1, maxY = -1;
    vector<vector<bool>> isOccupied;  // Keeps track of whether a cell is occupied by an enemy robot
    Battlefield* battlefield;

public:
    virtual ~SeeingRobot() {}
    SeeingRobot(Battlefield* bf) : battlefield(bf) {}

    // Getter
    Battlefield* getBattlefield() const {
        return battlefield;
    }

    //Setter
    void setBattlefield(Battlefield* bf) {
        battlefield = bf;
    }


    // Pure virtual function to be implemented by derived classes
    virtual void actionLook(int x, int y) = 0; // Reveals 9-square area, centered on (robotPositionX + x, robotPositionY + y)

    // Function to check the validity of a position on the battlefield
    bool isInBounds(int x, int y) {
        return x >= 0 && x < battlefield->BATTLEFIELD_NUM_OF_COLS() && y >= 0 && y < battlefield->BATTLEFIELD_NUM_OF_ROWS();
    }

    // Function to check if a location contains an enemy robot
    void processLookAround(int x, int y) {
        // Reset the grid each time the robot looks around
        isOccupied.clear();
        isOccupied.resize(3, vector<bool>(3, false)); // 3x3 grid for looking around

        // Set the boundaries for the look action
        minX = robotPositionX + x - 1;
        minY = robotPositionY + y - 1;
        maxX = robotPositionX + x + 1;
        maxY = robotPositionY + y + 1;

        // Loop through the 9-square area
        for (int i = minY; i <= maxY; ++i) {
            for (int j = minX; j <= maxX; ++j) {
                if (isInBounds(j, i)) {
                    // Mark if the location is occupied by an enemy robot
                    isOccupied[i - minY][j - minX] = isOccupiedByEnemy(j, i);
                }
            }
        }
    }

    // Function to check if a location contains an enemy robot
    bool isOccupiedByEnemy(int x, int y) {
    for (auto& robot : battlefield->getRobots()) {
        if (robot->x() == x && robot->y() == y && robot != this) {
            return true;  // If the robot at (x, y) is not the current robot, it's an enemy
        }
    }
    return false;
    }

// Function to get the result of the look action (whether an area is occupied or not)
    bool getOccupationStatus(int x, int y) {
        // Convert relative coordinates to absolute coordinates within the look area
        if (isInBounds(x + robotPositionX, y + robotPositionY)) {
            return isOccupied[y + 1][x + 1];  // +1 to adjust the 3x3 grid offset
        }
        return false;
    }

};


class ThinkingRobot : public virtual Robot{
protected:

public:
    virtual ~ThinkingRobot() {}
    virtual void think() = 0;  // Pure virtual function for decision-making
};


class ShootingRobot : virtual public Robot {
protected:
    int shells = 10;
    Battlefield* battlefield; // Add battlefield pointer to know battlefield state

public:
    ShootingRobot(Battlefield* bf = nullptr) : battlefield(bf) {}
    virtual ~ShootingRobot() {}

    virtual void actionShoot() = 0;

    bool fire(int targetX, int targetY) {
        if (targetX == this->x() && targetY == this->y()) {
            cout << "Robot cannot fire at its own position!" << endl;
            return false;
        }

        int hitProbability = rand() % 100;
        if (hitProbability < 70) {
            cout << "Hit! Robot fired at (" << targetX << ", " << targetY << ")." << endl;
            if (battlefield && battlefield->isOccupied(targetX, targetY)) {
                const auto& gridRef = battlefield->getGrid();
                string robotId = gridRef[targetY][targetX];
                for (auto& robot : battlefield->getRobots()) {
                    if (robot->id() == robotId) {
                        robot->setNumOfLives(robot->numOfLives() - 1);
                        if (robot->numOfLives() == 0) {
                            battlefield->eraseRobotById(robotId);
                            cout << "Robot " << robotId << " has been killed!" << endl;
                        } else {
                            cout << "Robot " << robotId << " survived, remaining lives: "
                                 << robot->numOfLives() << endl;
                        }

                        return true;
                    }
                }
            }
            return true;
        } else {
            cout << "Missed! Robot fired at (" << targetX << ", " << targetY << ")." << endl;
            return false;
        }
    }

    void chooseTargetAndFire() {
        int targetX = rand() % battlefield->BATTLEFIELD_NUM_OF_COLS();
        int targetY = rand() % battlefield->BATTLEFIELD_NUM_OF_ROWS();
        fire(targetX, targetY);
    }

    void actionFire() {
        chooseTargetAndFire();
    }
};


class HideBot : public MovingRobot, public ShootingRobot {
private:
    int hideCount = 0;      // Track how many times the robot has hidden
    const int maxHide = 3;  // Max times the robot can hide
    bool isHidden = false;  // Flag to track if the robot is hidden

public:
    HideBot(Battlefield* bf) : ShootingRobot(bf) {
        hideCount = 0;
        isHidden = false;
    }

    // Implement actionMove function for HideBot
    void actionMove() override {
        if (isHidden) {
            cout << "HideBot is hidden and cannot move!" << endl;
        } else {
            cout << "HideBot is moving!" << endl;
            // Example movement logic: random move
            int newX = x() + (rand() % 3 - 1);
            int newY = y() + (rand() % 3 - 1);
            setLocation(newX, newY);
        }
    }

    // Implement actionFire function for HideBot
    void actionFire() {
        if (isHidden) {
            cout << "HideBot is hidden and cannot fire!" << endl;
        } else {
            cout << "HideBot is firing!" << endl;
            // Firing logic (you can use the logic from the base class or modify it)
            ShootingRobot::actionFire();
        }
    }

    // Hide the robot (can be used to prevent it from moving or firing)
    void actionHide() {
        if (hideCount < maxHide) {
            isHidden = true;
            hideCount++;
            cout << "HideBot is now hidden! Remaining hides: " << (maxHide - hideCount) << endl;
        } else {
            cout << "No more hides left for this HideBot!" << endl;
        }
    }

    // Unhide the robot (make it visible again)
    void actionUnhide() {
        isHidden = false;
        cout << "HideBot is now visible again." << endl;
    }

    void actionShoot() override{} // this is just a dummy function that is implemented just because ShootingRobot class has this method as pure virtual
};


class JumpBot : public MovingRobot {
private:
    Battlefield* battlefield;
    int jumpCount = 0;
    const int maxJumps = 3;

public:
    JumpBot(Battlefield* bf) : battlefield(bf) {}

    virtual ~JumpBot() {}

    // Provide the concrete implementation for actionMove
    void actionMove() override {
        if (jumpCount < maxJumps) {
            int randomX = rand() % battlefield->getCols();
            int randomY = rand() % battlefield->getRows();

            setLocation(randomX, randomY);
            battlefield->updateBattlefield();

            cout << "JumpBot jumped to: (" << randomX << ", " << randomY << ")" << endl;
            jumpCount++;
        } else {
            cout << "JumpBot has used all its jumps, moving normally." << endl;
            MovingRobot::actionMove();
        }
    }
};


class LongShotBot : public ShootingRobot {
public:

    LongShotBot(Battlefield* bf) : ShootingRobot(bf) {}
    virtual ~LongShotBot() {}

    // Override the actionFire method to implement long-range firing with range restrictions
    void actionShoot() override {
        // Define all 16 possible directions (including diagonal shots)
        const int directions[16][2] = {
            {-3,  0}, // Left (3 units left)
            {-2,  0}, // Left (2 units left)
            {-1,  0}, // Left (1 unit left)
            { 1,  0}, // Right (1 unit right)
            { 2,  0}, // Right (2 units right)
            { 3,  0}, // Right (3 units right)
            { 0,  1}, // Up (1 unit up)
            { 0, -1}, // Down (1 unit down)
            { 1,  2}, // Up-Right (1 unit right, 2 units up)
            { 2,  1}, // Right-Up (2 units right, 1 unit up)
            {-1,  2}, // Up-Left (1 unit left, 2 units up)
            {-2,  1}, // Left-Up (2 units left, 1 unit up)
            { 1, -2}, // Down-Right (1 unit right, 2 units down)
            { 2, -1}, // Right-Down (2 units right, 1 unit down)
            {-1, -2}, // Down-Left (1 unit left, 2 units down)
            {-2, -1}  // Left-Down (2 units left, 1 unit down)
        };

        std::vector<std::pair<int, int>> validTargets;  // Vector to store valid targets

        // Generate all possible target locations within the range (up to 3 units away)
        for (int i = 0; i < 16; ++i) {
            int targetX = robotPositionX + directions[i][0];
            int targetY = robotPositionY + directions[i][1];

            // Check if the target is within range: x + y <= 3
            if (std::abs(directions[i][0]) + std::abs(directions[i][1]) <= 3) {
                validTargets.push_back({targetX, targetY});
            }
        }

        // Check if there are valid targets
        if (validTargets.empty()) {
            std::cout << "No valid targets within range!" << std::endl;
            return;
        }

        // Select a random valid target from the list
        int randomIndex = rand() % validTargets.size();
        int selectedX = validTargets[randomIndex].first;
        int selectedY = validTargets[randomIndex].second;

        // Fire at the randomly selected valid target
        std::cout << "LongShotBot fired at target (" << selectedX << ", " << selectedY << ")!" << std::endl;

        // Simulate firing and checking for hits (can be expanded with your logic)
        fire(selectedX, selectedY);  // Calling fire to check the hit
    }

    // Ensure fire() method is defined or inherited correctly
    bool fire(int targetX, int targetY) {
        // You can call the base class fire method or implement it here
        int hitProbability = rand() % 100;  // Generate a random number between 0 and 99
        if (hitProbability < 70) {  // 70% chance to hit
            std::cout << "Shot hit at (" << targetX << ", " << targetY << ")." << std::endl;
            return true;  // Successful hit
        } else {
            std::cout << "Shot missed at (" << targetX << ", " << targetY << ")." << std::endl;
            return false;  // Missed shot
        }
    }
};




class SemiAutoBot : public ShootingRobot {
private:
    int shellsRemaining = 10;

public:
    SemiAutoBot(Battlefield* bf) : ShootingRobot(bf) {}

    virtual ~SemiAutoBot() {}

    // Provide the concrete implementation for actionFire
    void actionShoot() override {
        int targetX = rand() % battlefield->BATTLEFIELD_NUM_OF_COLS();
        int targetY = rand() % battlefield->BATTLEFIELD_NUM_OF_ROWS();

        cout << "SemiAutoBot is firing at target (" << targetX << ", " << targetY << ")\n";
        ShootingRobot::fire(targetX, targetY);
    }
};




class ThirtyShotBot : public ShootingRobot {
private:
    int shells = 30;

public:
    ThirtyShotBot(Battlefield* bf) : ShootingRobot(bf) {}

    virtual ~ThirtyShotBot() {}

    // Provide the concrete implementation for actionFire
    void actionShoot() override {
        for (int shot = 0; shot < 30; ++shot) {
            int targetX = rand() % battlefield->BATTLEFIELD_NUM_OF_COLS();
            int targetY = rand() % battlefield->BATTLEFIELD_NUM_OF_ROWS();

            cout << "ThirtyShotBot is firing at target (" << targetX << ", " << targetY << ")\n";
            ShootingRobot::fire(targetX, targetY);
        }
    }
};



class ScoutBot : public SeeingRobot {
private:
    int remainingScans = 3;

public:
    ScoutBot(Battlefield* bf) : SeeingRobot(bf) {}

    virtual ~ScoutBot() {}

    // Provide the concrete implementation for actionLook
    void actionLook(int xOffset, int yOffset) override {
        if (remainingScans > 0) {
            cout << "ScoutBot is scanning the battlefield!" << endl;
            SeeingRobot::processLookAround(xOffset, yOffset);
            remainingScans--;
            cout << "Remaining scans: " << remainingScans << endl;
        } else {
            cout << "No more scans left for ScoutBot." << endl;
        }
    }
};



class TrackBot : public SeeingRobot {
private:
    int remainingTrackers = 3;
    unordered_map<string, pair<int, int>> trackedRobots;

public:
    TrackBot(Battlefield* bf) : SeeingRobot(bf) {}

    virtual ~TrackBot() {}

    // Provide the concrete implementation for actionLook
    void actionLook(int xOffset, int yOffset) override {
        cout << "TrackBot is scanning the battlefield and planting trackers." << endl;
        SeeingRobot::processLookAround(xOffset, yOffset);
    }

    // Plant a tracker on a robot
    void plantTracker(Robot* robot) {
        if (remainingTrackers > 0) {
            trackedRobots[robot->id()] = {robot->x(), robot->y()};
            remainingTrackers--;
            cout << "Tracker planted on " << robot->id() << ". Remaining trackers: " << remainingTrackers << endl;
        } else {
            cout << "No more trackers left for TrackBot." << endl;
        }
    }

    // Retrieve the tracked robot's location
    void checkTrackedRobotLocation(const string& robotId) {
        if (trackedRobots.find(robotId) != trackedRobots.end()) {
            pair<int, int> location = trackedRobots[robotId];
            cout << "Tracked " << robotId << " at (" << location.first << ", " << location.second << ")." << endl;
        } else {
            cout << "No tracker found for robot " << robotId << endl;
        }
    }
};


// New updates

class FiftyShotBot : public ShootingRobot {
private:
    int shells = 50;  // 50 shells for this robot
    int kills = 0;    // Track the number of kills

public:
    virtual ~FiftyShotBot() {}

    // Override actionFire to simulate 50 shots, one per action
    void actionShoot() override {
        // Check if there are shells available
        if (shells > 0) {
            int targetX = rand() % battlefield->BATTLEFIELD_NUM_OF_COLS();
            int targetY = rand() % battlefield->BATTLEFIELD_NUM_OF_ROWS();
            std::cout << "FiftyShotBot is firing at target (" << targetX << ", " << targetY << ")\n";

            // Fire and decrement shells after each shot
            bool hit = fire(targetX, targetY);
            if (hit) {
                // Handle robot destruction logic (if hit)
                handleRobotDestruction(targetX, targetY);
            }

            shells--;  // Decrement shells after firing
            std::cout << "Shells remaining: " << shells << std::endl;

            // Reload if shells are used up
            if (shells == 0) {
                std::cout << "No shells left, reloading...\n";
                shells = 50;  // Reload to 50 shells
            }
        } else {
            std::cout << "Out of shells, reloading!\n";
            shells = 50;  // Automatically reload if shells are 0
        }
    }

    // Simulate firing at a target with 70% chance of hitting
    bool fire(int targetX, int targetY) {
        int hitProbability = rand() % 100;
        if (hitProbability < 70) {
            std::cout << "Shot hit at (" << targetX << ", " << targetY << ").\n";
            return true;  // Successful hit
        } else {
            std::cout << "Shot missed at (" << targetX << ", " << targetY << ").\n";
            return false;  // Missed shot
        }
    }

    // Handle robot destruction logic after a successful shot
    void handleRobotDestruction(int targetX, int targetY) {
        if (battlefield->isOccupied(targetX, targetY)) {
            std::cout << "Hit confirmed! Checking if there's a robot to destroy...\n";

            string robotId = battlefield->getGrid()[targetY][targetX];
            for (size_t i = 0; i < battlefield->getRobots().size(); ++i) {
                if (battlefield->getRobots()[i]->id() == robotId) {
                    battlefield->getRobots()[i]->setNumOfLives(battlefield->getRobots()[i]->numOfLives() - 1);
                    if (battlefield->getRobots()[i]->numOfLives() == 0) {
                        battlefield->eraseRobotById(robotId);
                        std::cout << "Robot " << robotId << " has been destroyed!" << std::endl;
                        addNumOfKills();
                    } else {
                        std::cout << "Robot " << robotId << " survived, remaining lives: "
                                  << battlefield->getRobots()[i]->numOfLives() << std::endl;
                    }
                    return;
                }
            }
        }
    }

    void addNumOfKills() {
        kills++;
        std::cout << "Kills: " << kills << std::endl;
    }
};



class XtraLongShotBot : public ShootingRobot {
public:
    virtual ~XtraLongShotBot() {}

    void actionShoot() override {
        // Define all 25 possible directions (including longer-range shots)
        const int directions[25][2] = {
            {-5,  0}, {-4,  0}, {-3,  0}, {-2,  0}, {-1,  0},  // Left range
            { 1,  0}, { 2,  0}, { 3,  0}, { 4,  0}, { 5,  0},  // Right range
            { 0,  1}, { 0, -1}, { 0,  2}, { 0, -2}, { 0,  3},  // Vertical range (up, down)
            { 1,  3}, { 2,  2}, { 3,  1}, { 4,  0}, { 5, -1},  // Diagonal range
            {-1,  3}, {-2,  2}, {-3,  1}, {-4,  0}, {-5, -1}   // Diagonal range (reverse)
        };

        std::vector<std::pair<int, int>> validTargets;

        // Generate all possible target locations within the range (up to 5 units away)
        for (int i = 0; i < 25; ++i) {
            int targetX = robotPositionX + directions[i][0];
            int targetY = robotPositionY + directions[i][1];

            // Add the valid target
            validTargets.push_back({targetX, targetY});
        }

        // Check if there are valid targets
        if (validTargets.empty()) {
            std::cout << "No valid targets within range!" << std::endl;
            return;
        }

        // Select a random valid target from the list
        int randomIndex = rand() % validTargets.size();
        int selectedX = validTargets[randomIndex].first;
        int selectedY = validTargets[randomIndex].second;

        std::cout << "XtraLongShotBot fired at target (" << selectedX << ", " << selectedY << ")!" << std::endl;

        fire(selectedX, selectedY);
    }

    bool fire(int targetX, int targetY) {
        int hitProbability = rand() % 100;
        if (hitProbability < 70) {
            std::cout << "Shot hit at (" << targetX << ", " << targetY << ")." << std::endl;
            return true;  // Successful hit
        } else {
            std::cout << "Shot missed at (" << targetX << ", " << targetY << ")." << std::endl;
            return false;  // Missed shot
        }
    }
};



class AutoTargetBot : public ShootingRobot {
public:
    virtual ~AutoTargetBot() {}

    void actionShoot() override {
        // Find the nearest enemy robot
        Robot* targetRobot = findNearestEnemy();
        if (targetRobot != nullptr) {
            int targetX = targetRobot->x();
            int targetY = targetRobot->y();
            std::cout << "AutoTargetBot is firing at target (" << targetX << ", " << targetY << ")." << std::endl;

            fire(targetX, targetY);
        } else {
            std::cout << "No enemies found!" << std::endl;
        }
    }

    Robot* findNearestEnemy() {
        int minDistance = INT_MAX;
        Robot* nearestRobot = nullptr;

        for (auto& robot : battlefield->getRobots()) {
            if (robot != this) {
                int distance = std::abs(robot->x() - this->x()) + std::abs(robot->y() - this->y());
                if (distance < minDistance) {
                    minDistance = distance;
                    nearestRobot = robot;
                }
            }
        }

        return nearestRobot;
    }

    bool fire(int targetX, int targetY) {
        int hitProbability = rand() % 100;
        if (hitProbability < 70) {
            std::cout << "Shot hit at (" << targetX << ", " << targetY << ")." << std::endl;
            return true;  // Successful hit
        } else {
            std::cout << "Shot missed at (" << targetX << ", " << targetY << ")." << std::endl;
            return false;  // Missed shot
        }
    }
};


class GenericRobot : public MovingRobot, public ShootingRobot, public SeeingRobot, public ThinkingRobot {
private:
    int shellsRemaining = 10;
    bool isHidden = false;
    int upgradeCount = 0;
    vector<string> availableUpgrades = { "Moving", "Shooting", "Seeing" };
    string CurrentUpgrade;
    Battlefield* battlefield;  // Store the battlefield pointer
    Robot* currentRobot; // Pointer to store the current robot type (GenericRobot, JumpBot, HideBot, etc.)

public:
    // Constructor: Accepts the battlefield pointer as a parameter
    GenericRobot(const string& name, int x, int y, Battlefield* bf)
        : SeeingRobot(bf), battlefield(bf)  // Initialize SeeingRobot and battlefield pointer
    {
        id_ = name;
        setLocation(x, y);  // Set the location of the robot
        isValid = true;      // Set the robot as valid
        currentRobot = nullptr;  // Initialize currentRobot pointer to nullptr
    }

    virtual ~GenericRobot() {
        // Clean up dynamically allocated memory if any
        if (currentRobot) {
            delete currentRobot;
        }
    }

    // Method for firing the weapon
    bool actionFire(int targetX, int targetY) {
        int hitProbability = rand() % 100;  // Calculate hit probability

        if (hitProbability < 70) {  // 70% chance to hit
            cout << "Hit! Robot fired at (" << targetX << ", " << targetY << ")." << endl;

            // Check if there is a robot at the target location
            if (battlefield && battlefield->isOccupied(targetX, targetY)) {
                const auto& gridRef = battlefield->getGrid();
                string robotId = gridRef[targetY][targetX];

                // Loop through robots to find and handle the one at the target location
                for (auto& robot : battlefield->getRobots()) {
                    if (robot->id() == robotId) {
                        robot->setNumOfLives(robot->numOfLives() - 1);  // Decrease the robot's life

                        // If the robot is destroyed, erase it from the battlefield
                        if (robot->numOfLives() == 0) {
                            battlefield->eraseRobotById(robotId);
                            cout << "Robot " << robotId << " has been killed!" << endl;
                        } else {
                            cout << "Robot " << robotId << " survived, remaining lives: "
                                 << robot->numOfLives() << endl;
                        }

                        return true;  // Successfully hit and handled the robot
                    }
                }
            }

            return true;  // Hit, but no robot at the target
        } else {
            cout << "Missed! Robot fired at (" << targetX << ", " << targetY << ")." << endl;
            return false;  // Missed the shot
        }
    }

    // Override actionMove() method from MovingRobot
    void actionMove() override {
        cout << id() << " is moving!" << endl;
        // Example implementation: random move (can be customized)
        int newX = x() + (rand() % 3 - 1);
        int newY = y() + (rand() % 3 - 1);
        setLocation(newX, newY);
    }

    // Override actionFire() from ShootingRobot
    void actionFire() {
        cout << id() << " is firing!" << endl;
        // Assuming ShootingRobot's actionFire is meant to simulate shooting:
        ShootingRobot::actionFire();
    }

    // Override actionLook() from SeeingRobot
    void actionLook(int xOffset, int yOffset) override {
        cout << id() << " is looking around!" << endl;  // Corrected cout statement
        // Example: call SeeingRobot look function
        SeeingRobot::processLookAround(xOffset, yOffset);  // Ensure this is defined in SeeingRobot
    }

    void actionShoot() override {} // dummy function to implement the pure virtual in ShootingRobot class

    void think() override {} // dummy function to implement the pure virtual in ThinkingRobot class

    // Implement actions to move, fire, and look
    //void actions() override {
    //    actionMove();
    //    actionFire();
    //    actionLook(0, 0);  // Default look offset
    //}

    // Upgrade logic for the GenericRobot
    void upgradeRobot() {
        if (upgradeCount < 3) {
            int randomUpgrade = rand() % availableUpgrades.size();
            string upgradeType = availableUpgrades[randomUpgrade];

            // Upgrade logic based on remaining available upgrades
            if (upgradeType == "Moving") {
                // Randomly upgrade to either HideBot or JumpBot
                int randChoice = rand() % 2;
                if (randChoice == 0) {
                    if (currentRobot) delete currentRobot;  // Clean up the old object if it's already assigned
                    currentRobot = new HideBot(battlefield); // Upgrade to HideBot
                } else {
                    if (currentRobot) delete currentRobot;
                    currentRobot = new JumpBot(battlefield); // Upgrade to JumpBot
                }
            } else if (upgradeType == "Shooting") {
                // Randomly upgrade to one of the ShootingRobot subclasses
                int randChoice = rand() % 3;
                if (randChoice == 0) {
                    if (currentRobot) delete currentRobot;
                    currentRobot = new SemiAutoBot(battlefield); // Upgrade to SemiAutoBot
                } else if (randChoice == 1) {
                    if (currentRobot) delete currentRobot;
                    currentRobot = new ThirtyShotBot(battlefield); // Upgrade to ThirtyShotBot
                } else {
                    if (currentRobot) delete currentRobot;
                    currentRobot = new LongShotBot(battlefield); // Upgrade to LongShotBot
                }
            } else if (upgradeType == "Seeing") {
                // Randomly upgrade to one of the SeeingRobot subclasses
                int randChoice = rand() % 2;
                if (randChoice == 0) {
                    if (currentRobot) delete currentRobot;
                    currentRobot = new ScoutBot(battlefield); // Upgrade to ScoutBot
                } else {
                    if (currentRobot) delete currentRobot;
                    currentRobot = new TrackBot(battlefield); // Upgrade to TrackBot
                }
            }

            // After upgrading, remove the used upgrade from the available list
            availableUpgrades.erase(availableUpgrades.begin() + randomUpgrade);
            upgradeCount++;

            cout << id() << " upgraded to " << upgradeType << "!" << endl;
        } else {
            cout << id() << " has reached the maximum upgrades!" << endl;
        }
    }
};


class Simulation {
private:
    Battlefield* battlefield;
    vector<Robot*> destroyedRobots;
    int totalTurns;
    int rows;
    int cols;

public:
    Simulation(int steps, int r, int c) : battlefield(nullptr), totalTurns(steps), rows(r), cols(c) {}

    void loadRobotsFromInput(const string& filename) {
        ifstream file(filename);
        string line;
        int rows, cols;
        int numRobots;

        getline(file, line);
        sscanf(line.c_str(), "M by N : %d %d", &rows, &cols);
        battlefield = new Battlefield(rows, cols);

        getline(file, line); // Skip the steps line
        getline(file, line);
        sscanf(line.c_str(), "robots: %d", &numRobots);

        for (int i = 0; i < numRobots; ++i) {
            string robotType, robotId;
            int x, y;
            getline(file, line);
            stringstream ss(line);
            ss >> robotType >> robotId >> x >> y;
            if (robotType == "GenericRobot") {
                GenericRobot* robot = new GenericRobot(robotId, x, y, battlefield);
                battlefield->addRobot(robot);
            }
        }
    }

    void runSimulation() {
        for (int turn = 0; turn < totalTurns; ++turn) {
            // Actions and movements for robots
            for (auto& robot : battlefield->getRobots()) {
                robot->actions();
                if (robot->numOfLives() == 0) {
                    destroyedRobots.push_back(robot);
                    battlefield->eraseRobotById(robot->id());
                }
            }

            // Re-enter one robot per turn
            if (!destroyedRobots.empty()) {
                Robot* robotToReenter = destroyedRobots.back();
                destroyedRobots.pop_back();
                Robot* newRobot = new GenericRobot(robotToReenter->id(), rand() % battlefield->BATTLEFIELD_NUM_OF_COLS(), rand() % battlefield->BATTLEFIELD_NUM_OF_ROWS());
                battlefield->addRobot(newRobot);
            }

            // Optionally print the battlefield state after each turn
            battlefield->updateBattlefield();
        }
    }
};




// Main function to run the simulation
int main() {
    srand(time(0));  // Seed for random number generation

    Simulation Simulation(300, 40, 50);
    Simulation.loadRobotsFromInput("input.txt");  // Load robots from input file

    Simulation.runSimulation();  // Run the simulation for the given steps

    return 0;
}



// its the final countdown
