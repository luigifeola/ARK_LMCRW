#ifndef LMCRWENVIRONMENT_CPP
#define LMCRWENVIRONMENT_CPP

#include "LMCRWEnvironment.h"

#include "kilobot.h"

#include <math.h>
#include <stdlib.h>
#include <random>

#include <QVector>
#include <QVector2D>
#include <QLineF>
#include <QDebug>
#include <QtMath>
#include <QColor>

namespace  {
    const double reachable_distance = (ARENA_SIZE/2) - (2.0*KILO_DIAMETER);
    const int num_sectors = 6;
    const QVector2D left_direction (1.0,0.0);
}

double mykilobotenvironment::normAngle(double angle){
    while (angle > 180) angle = angle - 360;
    while (angle < -180) angle = angle + 360;
    return angle;
}

QVector2D mykilobotenvironment::VectorRotation2D (double angle, QVector2D vec){
    // qDebug() << "2D Rotation";
    QVector2D rotated_vector;
    double kx = (cos(angle)* vec.x()) + (-1.0*sin(angle) * vec.y());
    double ky = (sin(angle) * vec.x()) + (cos(angle) * vec.y());
    rotated_vector.setX(kx);
    rotated_vector.setY(ky);
    return rotated_vector;
}

QVector<int> mykilobotenvironment::proximity_sensor(QVector2D obstacle_direction, double kilo_rotation, int num_bit){
    double sector = M_PI_2 / (num_bit/2.0);
    QVector<int> proximity;

    // qDebug() << "kilo_ori" << qRadiansToDegrees(kilo_rotation);
    for(int i=0; i<num_bit; i++)
    {
        QVector2D sector_dir_start = VectorRotation2D((kilo_rotation+M_PI_2 - i * sector), left_direction);
        QVector2D sector_dir_end = VectorRotation2D((kilo_rotation+M_PI_2 - (i+1) * sector), left_direction);

//         qDebug() << "wall-dir" << obstacle_direction;
//         qDebug() << "a-dir" << sector_dir_start;
//         qDebug() << "b-dir" << sector_dir_end;

        if( QVector2D::dotProduct(obstacle_direction, sector_dir_start) >=0 ||
            QVector2D::dotProduct(obstacle_direction, sector_dir_end) >=0    )
        {
            proximity.push_back(0);
        }
        else{
            proximity.push_back(1);
        }
    }

    return proximity;
}

mykilobotenvironment::mykilobotenvironment(QObject *parent) : KilobotEnvironment(parent) {
    // environment specifications
    // this->ArenaX = 0.45;
    // this->ArenaY = 0.45;

    this->saveLOG = false;

    std::default_random_engine re;
    const int kSeed = 349530199;
    re.seed(kSeed);
//    qDebug() << "qrand()" << qrand();
//    re.seed(qrand());

    // define environment:
    vTarget.tRadius = KILO_DIAMETER;
    vTarget.tColor = Qt::red;




    QPoint k_center((ARENA_CENTER + SHIFTX), (ARENA_CENTER + SHIFTY));

    //            std::uniform_int_distribution<int> distr_x(7.0*CM_TO_PIXEL, ARENA_SIZE/2 - 10*CM_TO_PIXEL);
    std::uniform_int_distribution<int> distr_x(0, reachable_distance - vTarget.tRadius);
    int distance = distr_x(re);
    qDebug() << "reachable_distance:" << reachable_distance;
    qDebug() << "distance:" << distance;
//    int distance = 650;

//    std::uniform_real_distribution<double> uniform_angle(-1.0, 1.0);
    std::uniform_real_distribution<double> uniform_angle(-1.0, 1.0);
    double theta = M_PI * uniform_angle(re);
    qDebug() << "theta:" << theta << "degrees:" << qRadiansToDegrees(theta);


//    vTarget.tPos = QPoint(ARENA_CENTER + SHIFTX + 400,1000);
    vTarget.tPos = k_center + QPoint(distance * cos(theta), distance * sin(theta));

    // call any functions to setup features in the environment (goals, homes locations and parameters).
    reset();
}



void mykilobotenvironment::reset(){
    this->time = 0;
    this->minTimeBetweenTwoMsg = 0;

    kilobots_states.clear();
    kilobots_states_LOG.clear();

    kilobots_positions.clear();
    kilobots_colours.clear();
    kilobots_in_collision.clear();

    isCommunicationTime = false;
    lastTransitionTime = this->time;



}

// Only update if environment is dynamic:
void mykilobotenvironment::update() {

}



// generate virtual sensors reading and send it to the kbs (same as for ARGOS)
/**
 * @brief mykilobotenvironment::updateVirtualSensor
 * @caption send message to the kilobot if:
 * 1) the experiment is not started -> send alpha & rho parameters
 * 2) if colliding -> collision avoidance message
 * 3) passing from NOT_TARGET_FOUND/COMMUNICATED to TARGET_FOUND
 */
void mykilobotenvironment::updateVirtualSensor(Kilobot kilobot_entity) {
    // qDebug() << QString("In updateVirtualSensor");

    // update local arrays
    kilobot_id k_id = kilobot_entity.getID();
    this->kilobots_positions[k_id] = kilobot_entity.getPosition();
    // update kilobot led colour (indicates the internal state of the kb)
    lightColour kb_colour = kilobot_entity.getLedColour();

    if(!kilobots_RED_check[k_id])
    {
        if(kb_colour == lightColour::RED && qRound(this->time*10.0f) >= 1.0f*10.0f ){
            this->kilobots_colours[k_id] = Qt::red;    // kilobot with information about the target

            kilobots_colours_time_check[k_id] = 10.0;

            if(this->kilobots_states[k_id] == NOT_TARGET_FOUND)
            {
                this->kilobots_states[k_id] = TARGET_COMMUNICATED;
                kilobots_RED_timer[k_id] = this->time;
                if(kilobots_sync_time[k_id] != -1)
                    qDebug() << "ERROR!!! SHOULD BE -1";
                kilobots_sync_time[k_id] = this->time;
                qDebug() << "New info for kID:" << k_id << " at time " << kilobots_sync_time[k_id];
            }
            else if(this->kilobots_states[k_id] == TARGET_COMMUNICATED &&
                    qRound( (this->time - kilobots_RED_timer[k_id])*10.0f) >= 20.0f*10.0f)
            {
                kilobots_RED_check[k_id] = true;
                qDebug() << "Always red kilobot:" << k_id;
            }
        }
        else
        {
            this->kilobots_colours[k_id] = Qt::black;   // random walking
            kilobots_RED_timer[k_id] = this->time;

            if (this->kilobots_states[k_id] == TARGET_COMMUNICATED)
            {
                if (kilobots_colours_time_check[k_id] == 0.0f)
                {
                    this->kilobots_states[k_id] = NOT_TARGET_FOUND;
                    kilobots_sync_time[k_id] = -1;
                    qDebug() << "Wrong colour assignment for kilobot: " << k_id;
                    kilobots_colours_time_check[k_id] = 10.0f;
                }
                else
                    kilobots_colours_time_check[k_id] -= 1.0f;
            }

        }
    }

//    if(kb_colour == lightColour::RED || this->kilobots_colours[k_id] == Qt::red){
//        this->kilobots_colours[k_id] = Qt::red;     // kilobot passed directly over the target
//    }
//    else if(kb_colour == lightColour::GREEN && this->kilobots_colours[k_id] != Qt::red){
//        this->kilobots_colours[k_id] = Qt::green;    // kilobot with information about the target
//        this->kilobots_states[k_id] = TARGET_COMMUNICATED;
//    }
//    else if(this->kilobots_colours[k_id] != Qt::red)
//    {
//        this->kilobots_colours[k_id] = Qt::black;   // random walking
//    }


    bool insideTarget = (pow(kilobots_positions[k_id].x() - vTarget.tPos.x(),2) + pow(kilobots_positions[k_id].y()-vTarget.tPos.y(),2))
                        <= (pow(vTarget.tRadius,2));
    if(insideTarget)
    {
//        qDebug() << k_id << " is inside the target!!!!";
        this->kilobots_states[k_id] = TARGET_FOUND;
        if(kilobots_fpt[k_id] == -1)
        {
            kilobots_fpt[k_id] = this->time;
            if(kilobots_sync_time[k_id] == -1)
                kilobots_sync_time[k_id] = this->time;
            qDebug() << "Target found for kID:" << k_id << " at time " << this->time;
        }
    }

    // qDebug() << QString("Sending message");
    // now we have everything up to date and everything we need
    // then if it is time to send the message to the kilobot send info to the kb
    if(!this->isCommunicationTime && this->time - this->lastSent[k_id] > minTimeBetweenTwoMsg){
        // send if
        // not_red + inside

        /* Prepare the inividual kilobot's message                   */
        /* see README.md to understand about ARK messaging           */
        /* data has 3x24 bits divided as                             */
        /*   ID 10b    type 4b  data 10b     <- ARK msg              */
        /*  data[0]   data[1]   data[2]      <- kb msg               */
        /* xxxx xxxx xxyy yyww wwww wwww     <- LMCRW                */
        /* x bits used for kilobot id                                */
        /* y bits used for type                                      */
        /* w bits used for TODO: for what?                           */

        kilobot_message message; // this is a 24 bits field not the original kb message
        // Prepare an empty ARK message
        message.id = 511;
        message.type = 1;
        message.data = 0;


        if(kilobots_states[k_id] == TARGET_FOUND)
        {
            message.id = k_id;

            // qDebug() << "time:"<<this->time << " ARK EXP MESSAGE to " << k_id << " INSIDE, type " << message.type;
            lastSent[k_id] = this->time;
            emit transmitKiloState(message);
        }

        /*******************************************************************************/
        /*******************WALL AVOIDANCE**********************************************/
        /*******************************************************************************/
        // store kb rotation toward the center if the kb is too close to the border
        // this is used to avoid that the kb gets stuck in the wall
        uint8_t proximity_decimal;  // 0 no turn

        QPoint k_center ((ARENA_CENTER+SHIFTX), (ARENA_CENTER+SHIFTY));
        // get position translated w.r.t. center of arena
        QVector2D k_pos = QVector2D(this->kilobots_positions[k_id]);

        bool colliding_backup = kilobots_in_collision[k_id];
        kilobots_in_collision[k_id] = pow(k_pos.x()-k_center.x(),2)+pow(k_pos.y()-k_center.y(),2) > (pow(reachable_distance,2));

        // get orientation (from velocity) in radians in [-3.14,3.14]
        QVector2D k_ori = QVector2D(kilobot_entity.getVelocity());
        k_ori.setX(k_ori.x()*10);
        k_ori.setY(k_ori.y()*10);
        // qDebug() << "Orientation: " << normAngle( qRadiansToDegrees(qAtan2(-k_ori.y(), k_ori.x())) );

        double k_rotation = qAtan2(-k_ori.y(), k_ori.x());
//        qDebug() << "orientation: " << normAngle( qRadiansToDegrees(k_rotation) );  //angolo in [-180, 180] gradi

        if(kilobots_in_collision[k_id])
        {
            double collision_angle = qAtan2(-1.0*(k_pos.y()-k_center.y()) , k_pos.x()-k_center.x());
            QVector2D collision_direction = QVector2D(reachable_distance*qCos(collision_angle+M_PI),reachable_distance*qSin(collision_angle+M_PI)).normalized();

//            qDebug() << "collision angle: " << qRadiansToDegrees(collision_angle) ;
//            qDebug() << "normalized" << collision_direction.normalized();
//            qDebug() << "direction rotated" << collision_direction;
            QVector<int> proximity = proximity_sensor(collision_direction, k_rotation, num_sectors);
            proximity_decimal = std::accumulate(proximity.begin(), proximity.end(), 0, [](int x, int y) { return (x << 1) + y; });

///**************Print collisions!!*/
//            if(!colliding_backup && colliding_backup != kilobots_in_collision[k_id] && proximity_decimal!=0)
//            {
//                qDebug() << this->time << ", " << k_id << " COLLIDING! -> " << proximity;
//            }
            message.id = k_id;
            message.type = 2;
            message.data = proximity_decimal;

            lastSent[k_id] = this->time;
            emit transmitKiloState(message);
        }


    }

#endif // LMCRWENVIRONMENT_CPP


}




