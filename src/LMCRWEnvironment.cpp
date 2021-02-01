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

mykilobotenvironment::mykilobotenvironment(QObject *parent) : KilobotEnvironment(parent) {
    // environment specifications
    // this->ArenaX = 0.45;
    // this->ArenaY = 0.45;

    this->saveLOG = false;

    // define environment:
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

    // qDebug() << QString("saving colours");
    // update kilobot led colour (indicates the internal state of the kb)
    lightColour kb_colour = kilobot_entity.getLedColour();
    if(kb_colour == lightColour::RED){
        this->kilobots_colours[k_id] = Qt::red;     // kilobot in WAITING
        // qDebug() << "ReeEEEEEEEEEEEEEEEEEEEEE " << k_id;
    }
    else if(kb_colour == lightColour::BLUE){
        this->kilobots_colours[k_id] = Qt::blue;    // kilobot in LEAVING
        // qDebug() << "BLUEEEEEEEEEEEEEEEEEEEEE " << k_id;
    }
    else
    {
        this->kilobots_colours[k_id] = Qt::black;   // random walking
        // qDebug() << "BLack****************** " << k_id;
    }



    // check if inside

    // qDebug() <<QString("Kilobot %1 state is: %2").arg(k_id).arg(kilobots_states[k_id]);
    // qDebug() <<QString("Kilobot %1 LOG state is: %2").arg(k_id).arg(kilobots_states_LOG[k_id]);



    // qDebug() << QString("Sending message");
    // now we have everything up to date and everything we need
    // then if it is time to send the message to the kilobot send info to the kb
    if(this->time - this->lastSent[k_id] > minTimeBetweenTwoMsg){
        // send if
        // not_red + inside

        /* Prepare the inividual kilobot's message                   */
        /* see README.md to understand about ARK messaging           */
        /* data has 3x24 bits divided as                             */
        /*   ID 10b    type 4b  data 10b     <- ARK msg              */
        /*  data[0]   data[1]   data[2]      <- kb msg               */
        /* xxxx xxxx xxyy yyww wwww wwww     <- dhtf                 */
        /* x bits used for kilobot id                                */
        /* y bits used for type                                      */
        /* w bits used for TODO: for what?                           */

        kilobot_message message; // this is a 24 bits field not the original kb message
        // make sure to start clean
        message.id = 0;
        message.type = 0;
        message.data = 0;


        if( (kilobots_states[k_id] == TARGET_FOUND) && kilobots_colours[k_id] != Qt::red)
        {
            message.id = k_id;

            // qDebug() << "time:"<<this->time << " ARK EXP MESSAGE to " << k_id << " INSIDE, type " << message.type;
            lastSent[k_id] = this->time;
            emit transmitKiloState(message);
        }


//        else
//            qDebug() << QString("NOT need to send a message to kilobot %1").arg(k_id) << endl;


    }

#endif // LMCRWENVIRONMENT_CPP


}




