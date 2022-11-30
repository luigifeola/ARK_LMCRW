#ifndef LMCRWENVIRONMENT_H
#define LMCRWENVIRONMENT_H

/**
 * Author: Luigi Feola
 *
 * This is the code that specifies the specific environment used for the LMCRW experiment.
 * The environment is a circular arena of 0.95 m, in which kilobots have to find and communicate
 * if found the target (a circle of diameter equals to 2 kilo_diameter)
 */
#include <QObject>
#include <QPointF>
#include <QVector>
#include <QVector3D>
#include <QTime>
#include <QMatrix>
#include <QList>
#include <QColor>
#include <QElapsedTimer>

#include <limits>
#include <bitset>

#include <kilobotenvironment.h>

#define SCALING 0.5
// #define SHIFTX 0 //sheffield
// #define SHIFTY 1000 //sheffield
#define SHIFTX 50 //cnr
#define SHIFTY 50 //cnr
#define ARENA_SIZE 1875
#define ARENA_CENTER ARENA_SIZE/2
#define KILO_DIAMETER 66 //cnr
// #define KILO_DIAMETER 33 //sheffield

typedef enum {
    NOT_TARGET_FOUND = 0,
    TARGET_FOUND = 1,
    TARGET_COMMUNICATED = 2,
    BIASING = 3,
    COLLIDING = 4
}kilobot_state;

class mykilobotenvironment : public KilobotEnvironment
{
    Q_OBJECT
public:
    double debug_period = 0.5;
    double lastDebug = 0.0;

    explicit mykilobotenvironment(QObject *parent = 0);
    void reset();



    QVector<kilobot_state> kilobots_states; // list of all kilobots locations meaning 0 for outside areas, 1 for inside
    QVector<kilobot_state> kilobots_states_LOG;
    QVector<QPointF> kilobots_positions;    // list of all kilobots positions
    QVector<QColor> kilobots_colours;  // list of all kilobots led colours, the led indicate the state of the kilobot
    QVector<double> kilobots_colours_time_check;  // list of all kilobots led colours duration check
    QVector<bool> kilobots_RED_check;  // if red for enough, always red!
    QVector<double> kilobots_RED_timer;  // timer for red led
    QVector<bool> kilobots_in_collision; //true at i if kilobot i is in collision


    QVector<float> lastSent;    // when the last message was sent to the kb at given position

    QVector<float> kilobots_fpt;          // when the kilobot crosses the target for the first time
    QVector<float> kilobots_sync_time;    // when the kilobot receives info about the target for the first time


    // int ArenaX, ArenaY;

    // used to implement the mechanism that switch between exploration and communication (i.e., robots not moving, experiments frozen)
    double lastTransitionTime; // used to switch between exploration time and communication time
    bool isCommunicationTime; // determine if the robots are communicating or explorations

    int exploration_time = 3;     //in seconds
    int communication_time = 1;   //in seconds

    float minTimeBetweenTwoMsg; // minimum time between two messages
    double time;
    bool saveLOG;


    struct VirtualTarget
    {
        QPoint tPos;
        double tRadius;
        QColor tColor;
    };

    VirtualTarget vTarget;

// signals and slots are used by qt to signal state changes to objects
signals:
    void errorMessage(QString);

public slots:
    void update();
    void updateVirtualSensor(Kilobot kilobot);


private:

    double normAngle(double angle); //map the angle in [-180,180]
    QVector2D VectorRotation2D (double angle, QVector2D vec);
    QVector<int> proximity_sensor(QVector2D obstacle_direction, double kilo_rotation, int num_bit);
};

#endif //LMCRWENVIRONMENT_H
