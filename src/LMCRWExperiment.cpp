#include "LMCRWExperiment.h"
#include "LMCRWEnvironment.h"

#include <QDebug>
#include <QThread>

// widgets
#include <QPushButton>
#include <QLineEdit>
#include <QHBoxLayout>
#include <QCheckBox>
#include <QScrollBar>
#include <QSlider>
#include <QGroupBox>
#include <QFormLayout>
#include <QLabel>
#include <QFrame>
#include <QtMath>
#include <QRadioButton>
#include <QPainter>
#include <QList>
#include <iterator>
#include <QSignalMapper>
#include <QFile>
#include <QDir>

#define STOP_AFTER 1810
//#define STOP_AFTER 310

// return pointer to interface!
// mykilobotexperiment can and should be completely hidden from the application
extern "C" LMCRWEXPSHARED_EXPORT KilobotExperiment *createExpt()
{
    return new mykilobotexperiment();
}

/* setup the environment */
mykilobotexperiment::mykilobotexperiment() {
//     qDebug() << QString("in constructor");

    // Initialize seed
    QDateTime cd = QDateTime::currentDateTime();
    qsrand(cd.toTime_t());

    // setup the environments here
    connect(&LMCRWEnvironment,SIGNAL(transmitKiloState(kilobot_message)), this, SLOT(signalKilobotExpt(kilobot_message)));
    this->serviceInterval = 100; // timestep expressed in ms

}




/* create the GUI as a separate frame in the main ARK window */
QWidget *mykilobotexperiment::createGUI() {
//     qDebug() << QString("in create gui");

    QFrame *frame = new QFrame;
    QVBoxLayout *lay = new QVBoxLayout;
    frame->setLayout(lay);

    // add check box for saving the images
    QCheckBox *saveImages_ckb = new QCheckBox("Record experiment");
    saveImages_ckb->setChecked(true);  // start as checked
    lay->addWidget(saveImages_ckb);
    toggleSaveImages(saveImages_ckb->isChecked());

    // add check box for logging the experiment
    QCheckBox *logExp_ckb = new QCheckBox("Log experiment");
    logExp_ckb->setChecked(true);   // start as checked
    lay->addWidget(logExp_ckb);
    toggleLogExp(logExp_ckb->isChecked());

    connect(saveImages_ckb, SIGNAL(toggled(bool)),this, SLOT(toggleSaveImages(bool)));
    connect(logExp_ckb, SIGNAL(toggled(bool)),this, SLOT(toggleLogExp(bool)));
    connect(this,SIGNAL(destroyed(QObject*)), lay, SLOT(deleteLater()));


    return frame;
}

void mykilobotexperiment::initialise(bool isResume) {
//    qDebug() << QString("in initialise");

    // generate the environments
     setupEnvironments();

    // Initialize Kilobot States:
    if (!isResume) {
        // init stuff
        emit getInitialKilobotStates();
    } else {
        // probably nothing
    }

    //POS = position, ROT = orientation, LED = color
    emit setTrackingType(POS | ROT | LED);

    QThread::currentThread()->setPriority(QThread::HighestPriority);

    savedImagesCounter = 0;
    this->time = 0;
    m_elapsed_time.start();

    // init log file operations
    // if the log checkmark is marked then save the logs
    if(logExp) {



        /********************************LOG FOR VIDEO***************************************************************************/
        // open file
        if(log_file.isOpen()) {
            // if it was open close and re-open again later
            // this erase the old content
            log_file.close();
        }
        // log filename consist of the prefix and current date and time
        QString log_filename = log_filename_prefix + "_kilopos.txt";
        log_file.setFileName(log_filename);
        // open the file
        if(log_file.open(QIODevice::WriteOnly)) {
            qDebug() << "Log file " << log_file.fileName() << " opened";
            log_stream.setDevice(&log_file);
//            log_stream
//                    << "time" << '\t'
//                    << "kID" << '\t'
//                    << "colour" << '\t'
//                    << "positionX" << '\t'
//                    << "positionY" << '\t'
//                    << "orientation" << '\t'
//                    << "state" << '\n';
            //Initial state
           log_stream << this->time;
           for(int i=0; i<kilobots.size();i++){
                log_stream << "\t"
                           << kilobots[i].id << '\t'
                           << kilobots[i].colour << '\t'
                           << kilobots[i].position.x() << '\t'
                           << kilobots[i].position.y() << '\t'
                           << kilobots[i].orientation <<'\t'
                           << kilobots[i].state;
            }
            log_stream << endl;

        }
        else {
            qDebug() << "ERROR opening file "<< log_filename;
        }

        // open file
        if(log_file1.isOpen()) {
            // if it was open close and re-open again later
            // this erase the old content
            log_file1.close();
        }
        // log filename consist of the prefix and current date and time
        log_filename = log_filename_prefix + "_targetpos.txt";
        log_file1.setFileName(log_filename);
        // open the file
        if(log_file1.open(QIODevice::WriteOnly)) {
            qDebug() << "Log file " << log_file1.fileName() << " opened";
            log_stream1.setDevice(&log_file1);
//            log_stream1
//                    << "time" << '\t'
//                    << "id" << '\t'
//                    << "positionX" << '\t'
//                    << "positionY" << '\t'
//                    << "colour" << '\t'
//                    << "state" << '\t'
//                    << "kilobots_in_area" << '\n';
            //Initial state
            log_stream1 << LMCRWEnvironment.vTarget.tPos.x() << '\t'
                        << LMCRWEnvironment.vTarget.tPos.y() << '\t'
                        << LMCRWEnvironment.vTarget.tRadius << '\n';
        }
        else {
            qDebug() << "ERROR opening file "<< log_filename;
        }

        /**************************************************************************************/
        /**LOG for first passage time & convergence time***************************************/
        /**************************************************************************************/
        // open file
        if(log_fileTime.isOpen()) {
            // if it was open close and re-open again later
            // this erase the old content
            log_fileTime.close();
        }
        // log filename consist of the prefix and current date and time
        log_filename = log_filename_prefix + "_time_results.txt";
        log_fileTime.setFileName(log_filename);

        // open the file
        if(log_fileTime.open(QIODevice::WriteOnly)) {
            qDebug() << "Log file " << log_fileTime.fileName() << " opened";
            log_streamTime.setDevice(&log_fileTime);
//            log_streamTime
//                    << "time" << '\t'
//                    << "id" << '\t'
//                    << "positionX" << '\t'
//                    << "positionY" << '\t'
//                    << "colour" << '\t'
//                    << "state" << '\t'
//                    << "kilobots_in_area" << '\n';
        }
        else {
            qDebug() << "ERROR opening file "<< log_filename;
        }

    }

    // if the checkbox for saving the images is checked
    if(saveImages) {
        emit saveImage(QString("./images/LMCRW_%1.jpg").arg(savedImagesCounter++, 5, 10, QChar('0')));
    }

    // clear old drawings (e.g., from ID-identification)
    clearDrawings();
}

void mykilobotexperiment::stopExperiment() {

    if (log_file.isOpen()){
        log_file.close();
    }
    //Close Log file
    if (log_file1.isOpen()){
        log_file1.close();
    }
    //Close Log file
    if (log_fileTime.isOpen()){
        log_fileTime.close();
    }
}

void mykilobotexperiment::run() {
    //qDebug() << QString("in run");


    this->time=m_elapsed_time.elapsed()/1000.0; // time in seconds

    // stop after given time
    if(this->time >= STOP_AFTER) {

        // LOG fpt and sync_time
        for(int i=0; i<LMCRWEnvironment.kilobots_sync_time.size();i++)
        {
            qDebug() << "kID:" << i << " sync_time:" << LMCRWEnvironment.kilobots_sync_time[i] << " fpt:" << LMCRWEnvironment.kilobots_fpt[i];

            log_streamTime << i << '\t'
                           << LMCRWEnvironment.kilobots_fpt[i] << '\t'
                           << LMCRWEnvironment.kilobots_sync_time[i] << '\n';
        }


        // close the experiment
        this->stopExperiment();
        emit(experimentComplete());
    }


    // Update Environment
    LMCRWEnvironment.time = (float)time;


    // Broadcast START signal and LMCRW parameters to the kilobots
    if(this->time <= 2.0)
    {
        kilobot_broadcast message;
        message.type = (int)START;
        message.data.resize(2);
        message.data[0] = 9;    //rho parameter
        message.data[1] = 20;   //alpha parameter
//        qDebug() << "Sending alpha: " << message.data[1] << " rho: " << message.data[0] << "\n";
        emit broadcastMessage(message);
    }

    if(true)
    {
        // switch between communication time and exploration time
        if(!LMCRWEnvironment.isCommunicationTime && LMCRWEnvironment.exploration_time <= this->time - LMCRWEnvironment.lastTransitionTime)
        {
            LMCRWEnvironment.isCommunicationTime = true;
            LMCRWEnvironment.lastTransitionTime = this->time;
            kilobot_broadcast message;
            message.type = (int)COMMUNICATION;
            emit broadcastMessage(message);
        }
        else if(LMCRWEnvironment.isCommunicationTime && LMCRWEnvironment.communication_time <= this->time - LMCRWEnvironment.lastTransitionTime)
        {
            LMCRWEnvironment.isCommunicationTime = false;
            LMCRWEnvironment.lastTransitionTime = this->time;
            kilobot_broadcast message;
            message.type = (int)STOP_COMMUNICATION;
            emit broadcastMessage(message);
        }

        // emit continuosly the "COMMUNICATION" or "sSTOP_COMMUNICATION" message
        // do this few times per second to avoid interferences
        if(uint(this->time*10)%10 == 0) {
            kilobot_broadcast message;
            message.type = LMCRWEnvironment.isCommunicationTime?(int)COMMUNICATION:(int)STOP_COMMUNICATION;
            emit broadcastMessage(message);
        }
    }

    // update kilobots states
    emit updateKilobotStates();

    // update visualization twice per second
    if( qRound((this->time-last_env_update)*10.0f) >= env_update_period*10.0f ) {
        // clear current environment
        last_env_update = this->time;
        clearDrawings();
        clearDrawingsOnRecordedImage();

        // plot updated environment
        plotEnvironment();
    }

    // save LOG files and images for videos
    if( logExp && (qRound((this->time - last_log)*10.0f) >= log_period*10.0f))
    {
        // qDebug() << "Log time: " << this->time <<" at " << QLocale("en_GB").toString( QDateTime::currentDateTime(), "hh:mm:ss.zzz");
        // qDebug() << "LOGs saving at " << this->time*10;
        last_log = this->time;
        if(saveImages) {
            // qDebug() << "Saving Image";
            emit saveImage(QString("./images/LMCRW_%1.jpg").arg(savedImagesCounter++, 5, 10, QChar('0')));
        }
        if(logExp)
        {
            log_stream << this->time;
            for(int i=0; i<kilobots.size();i++)
            {
                 log_stream << "\t"
                            << kilobots[i].id << '\t'
                            << kilobots[i].colour << '\t'
                            << kilobots[i].position.x() << '\t'
                            << kilobots[i].position.y() << '\t'
                            << kilobots[i].orientation <<'\t'
                            << kilobots[i].state;
            }
            log_stream << endl;


        }
    }


}

// Setup the Initial Kilobot Environment:
//   This is run once for each kilobot after emitting getInitialKilobotStates() signal.
//   This assigns kilobots to an environment.
void mykilobotexperiment::setupInitialKilobotState(Kilobot kilobot_entity) {
//    qDebug() << QString("in setup init kilobot state");

    // assign all kilobot to environment LMCRW
    this->setCurrentKilobotEnvironment(&LMCRWEnvironment);
    kilobot_id k_id = kilobot_entity.getID();

    // create a necessary list and variable for correct message timing
    if(k_id+1 > kilobots.size()) {
        kilobots.resize(k_id+1);
    }

    // this are used to solve a bug causing the app to crash after resetting
    if(LMCRWEnvironment.lastSent.size() < k_id+1) {
        LMCRWEnvironment.lastSent.resize(k_id+1);
    }
    if(LMCRWEnvironment.kilobots_positions.size() < k_id+1) {
        LMCRWEnvironment.kilobots_positions.resize(k_id+1);
    }
    if(LMCRWEnvironment.kilobots_states.size() < k_id+1) {
        LMCRWEnvironment.kilobots_states.resize(k_id+1);
    }
    if(LMCRWEnvironment.kilobots_states_LOG.size() < k_id+1) {
        LMCRWEnvironment.kilobots_states_LOG.resize(k_id+1);
    }
    if(LMCRWEnvironment.kilobots_colours.size() < k_id+1) {
        LMCRWEnvironment.kilobots_colours.resize(k_id+1);
    }
    if(LMCRWEnvironment.kilobots_in_collision.size() < k_id+1) {
        LMCRWEnvironment.kilobots_in_collision.resize(k_id+1);
    }


    if(LMCRWEnvironment.kilobots_fpt.size() < k_id+1) {
        LMCRWEnvironment.kilobots_fpt.resize(k_id+1);
        LMCRWEnvironment.kilobots_fpt.fill(-1);
    }
    if(LMCRWEnvironment.kilobots_sync_time.size() < k_id+1) {
        LMCRWEnvironment.kilobots_sync_time.resize(k_id+1);
        LMCRWEnvironment.kilobots_sync_time.fill(-1);
    }
    if(LMCRWEnvironment.kilobots_colours_time_check.size() < k_id+1) {
        LMCRWEnvironment.kilobots_colours_time_check.resize(k_id+1);
        LMCRWEnvironment.kilobots_colours_time_check.fill(0);
    }
    if(LMCRWEnvironment.kilobots_RED_check.size() < k_id+1) {
        LMCRWEnvironment.kilobots_RED_check.resize(k_id+1);
        LMCRWEnvironment.kilobots_RED_check.fill(0);
    }
    if(LMCRWEnvironment.kilobots_RED_timer.size() < k_id+1) {
        LMCRWEnvironment.kilobots_RED_timer.resize(k_id+1);
        LMCRWEnvironment.kilobots_RED_timer.fill(false);
    }





    LMCRWEnvironment.lastSent[k_id] = LMCRWEnvironment.minTimeBetweenTwoMsg;

    // TODO initialize kilobots location correctly
    LMCRWEnvironment.kilobots_positions[k_id] = kilobot_entity.getPosition();
    LMCRWEnvironment.kilobots_states[k_id] = NOT_TARGET_FOUND;
    LMCRWEnvironment.kilobots_states_LOG[k_id] = NOT_TARGET_FOUND;
    LMCRWEnvironment.kilobots_colours[k_id] = Qt::black;
    LMCRWEnvironment.kilobots_in_collision[k_id] = false;

    KiloLog kLog(k_id, kilobot_entity.getPosition(), 0, kilobot_entity.getLedColour());
    kLog.state=NOT_TARGET_FOUND;
    kilobots[k_id] = kLog;
    // qDebug() << "ORIENTATION IS " << kilobots[k_id].orientation;
    if(!kilobots_ids.contains(k_id))
        kilobots_ids.append(k_id);

    double timeForAMessage = 0.05; // 50 ms each message
    LMCRWEnvironment.minTimeBetweenTwoMsg = kilobots_ids.size()*timeForAMessage/2.8;
    LMCRWEnvironment.lastSent[k_id] = LMCRWEnvironment.minTimeBetweenTwoMsg;
    // qDebug() << "Min time between two messages is" << LMCRWEnvironment.minTimeBetweenTwoMsg;

}

// run once for each kilobot after emitting updateKilobotStates() signal
void mykilobotexperiment::updateKilobotState(Kilobot kilobotCopy) {
//    qDebug() << QString("in update KilobotStates");

    // update values for logging
    if(logExp) 
    {
        kilobot_id k_id = kilobotCopy.getID();
        kilobot_colour k_colour = kilobotCopy.getLedColour();
        QPointF k_position = kilobotCopy.getPosition();
        double k_rotation = qRadiansToDegrees(qAtan2(-kilobotCopy.getVelocity().y(), kilobotCopy.getVelocity().x()));
        kilobot_state k_state = LMCRWEnvironment.kilobots_states[k_id];
        kilobots[k_id].updateAllValues(k_id, k_position, k_rotation, k_colour, k_state);
    }
}

// Setup Environment:
void mykilobotexperiment::setupEnvironments( ) {
//    qDebug() << QString("in setup env");
    LMCRWEnvironment.reset();
}

//WARNING : what is the goal of the GetFloorColor function?
QColor mykilobotexperiment::GetFloorColor(int track_x, int track_y) {
//    qDebug() << QString("in get floor color");

    QColor floorColour = Qt::white; // no resource
    QPoint env_point = QPoint(track_x,track_y);

    if ( pow(env_point.x()-LMCRWEnvironment.vTarget.tPos.x(),2)+pow(env_point.y()-LMCRWEnvironment.vTarget.tPos.y(),2) <= (pow(LMCRWEnvironment.vTarget.tRadius,2)) )
    {
        floorColour = LMCRWEnvironment.vTarget.tColor;
    }

    // paint the resources

    return floorColour;
}

// Plot Environment on frame:
void mykilobotexperiment::plotEnvironment() {
//    qDebug() << QString("In plot environment");
    // clean image
    clearDrawingsOnRecordedImage();


//    //Draw the Arena
//    drawCircle(QPointF(ARENA_CENTER+SHIFTX,ARENA_CENTER+SHIFTY), ARENA_SIZE/2, QColor(Qt::yellow), 10, "", false);

    //Draw the reachable position
    drawCircle(QPointF(ARENA_CENTER+SHIFTX,ARENA_CENTER+SHIFTY), ((ARENA_SIZE/2) - (2.0*KILO_DIAMETER)), QColor(Qt::red), 5, "", false);
    //Draw the target
    drawCircle(LMCRWEnvironment.vTarget.tPos, LMCRWEnvironment.vTarget.tRadius, LMCRWEnvironment.vTarget.tColor, 10, "", false);

    /**
     * Draw some useful position
    **/

    // Center
//    QPoint k_center ((ARENA_CENTER+SHIFTX), (ARENA_CENTER+SHIFTY));
//    drawCircle(QPoint(k_center.x(),k_center.y()), 10.0, QColor(Qt::black), 10, "", false);

    /* x and y axis */
//    std::vector<cv::Point> xAx {Point(SHIFTX, ARENA_SIZE/2+SHIFTY), Point(ARENA_SIZE+SHIFTX, ARENA_SIZE/2+SHIFTY)};
//    drawLine(xAx,Qt::black, 2,"",false);
//    std::vector<cv::Point> yAx {Point((ARENA_SIZE/2+SHIFTX), SHIFTY), Point((ARENA_SIZE/2)+SHIFTX, ARENA_SIZE+SHIFTY)};
//    drawLine(yAx,Qt::black, 2,"",false);

    /* Draw reachable distance */
//    std::vector<cv::Point> reachable_segment {cv::Point(ARENA_CENTER+SHIFTX,ARENA_CENTER+SHIFTY), cv::Point(((ARENA_SIZE) - (3.0/2.0*KILO_DIAMETER))+SHIFTX , ARENA_CENTER+SHIFTY)};
//    drawLine(reachable_segment,Qt::black, 5,"",false);

//    QPoint k_reachable (((ARENA_SIZE) - (3.0/2.0*KILO_DIAMETER))+SHIFTX, (ARENA_CENTER+SHIFTY));
//    drawCircle(QPoint(k_reachable.x(),k_reachable.y()), 20.0, QColor(Qt::yellow), 15, "", false);

    /*collision segment*/
//    int reachable_distance = ((ARENA_SIZE) - (3.0/2.0*KILO_DIAMETER))+SHIFTX - (ARENA_CENTER+SHIFTX);
//    double coll_x = reachable_distance * qCos(-0.477013) + k_center.x();
//    double coll_y = -1.0 * reachable_distance * qSin(-0.477013) + k_center.y();
//    std::vector<cv::Point> collision_segment {cv::Point(ARENA_CENTER+SHIFTX,ARENA_CENTER+SHIFTY), cv::Point(coll_x,coll_y)};
//    drawLine(collision_segment,Qt::black, 5,"collision",false);

    // Draw circles on recorded images
    // if(this->saveImages)
    // {
    //     if(!a->completed)
    //         drawCircleOnRecordedImage(a->position, a->radius, a->color, 10, std::to_string(a->id));
    //     else
    //         drawCircleOnRecordedImage(a->position, a->radius, Qt::transparent, 10, std::to_string(a->id));
    // }


    // Draw led on kilobots
    // for(uint k_id : this->kilobots_ids) {
    //     drawCircleOnRecordedImage(kilobots[k_id].position, 5, kilobots[k_id].colour, 5, "");
    // }
}


