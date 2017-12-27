#include <iostream>
#include <unistd.h>

#include "maindialog.h"
#include "ui_maindialog.h"
#include "selectcamerasdialog.h"
#include "../libviso2/src/timer.h"

#define MAIN_DIALOG_DEBUG 0

MainDialog::MainDialog(QWidget *parent) :
    QDialog(parent),
    _ui(new Ui::MainDialog)
{
    setWindowFlags(Qt::WindowMinMaxButtonsHint);
    _ui->setupUi(this);
    _stereo_image = new StereoImage();
    _time_of_last_frame.tv_sec  = 0;
    _time_of_last_frame.tv_usec = 0;
    _calib = new CalibIOKITTI();
    _capture_mutex = new QMutex();
    //cam_left      = new FrameCaptureThread(_stereo_image,_calib,true,_capture_mutex);
    //cam_right     = new FrameCaptureThread(_stereo_image,_calib,false,_capture_mutex);
    _visualOdomThread     = new VisualOdometryThread(_calib);
    _stereo_thread = new StereoThread(_calib,_ui->modelView);
    _stereo_image_io = new StereoImageIOKITTI();
    _oxts_io = new OxTSIOKITTI();
    _read_thread   = new ReadFromFilesThread(_stereo_image, _calib, _stereo_image_io, _oxts_io);
    _visualize_thread = new VisualizeThread(_ui->disparityView,_ui->modelView);

    // connect to the objects for communication.
    QObject::connect(_stereo_image,SIGNAL(newStereoImageArrived()),this,SLOT(newStereoImageArrived()));
    //QObject::connect(_visualOdomThread,SIGNAL(newHomographyArrived()),this,SLOT(newHomographyArrived()));
    //QObject::connect(_stereo_thread,SIGNAL(newDisparityMapArrived()),this,SLOT(newDisparityMapArrived()));
    QObject::connect(_calib, SIGNAL( newCalibrationData() ), this, SLOT( onNewCalibrationData() ) );

    _frame_index = 0;
    _gain_total   = 1.0F;
    _stereo_scan  = false;
    _settings = new QSettings("KIT", "stereomapper");
    QPalette p(palette());
    p.setColor(QPalette::Background, Qt::white);
    setPalette(p);

    _ui->shutterSpinBox->setValue(_settings->value("shutter_value","100").toInt());
    _ui->modelView->setBackgroundWallFlag(_ui->backgroundWallCheckBox->isChecked());
    _ui->modelView->setShowCamerasFlag(_ui->showCamerasCheckBox->isChecked());
    _ui->modelView->setGridFlag(_ui->gridCheckBox->isChecked());
    _ui->modelView->setWhiteFlag(_ui->whiteCheckBox->isChecked());
}

//==============================================================================//

MainDialog::~MainDialog()
{
    delete _ui;
    //delete cam_left;
    //delete cam_right;
    delete _capture_mutex;
    delete _stereo_image;
    delete _calib;
    delete _visualOdomThread;
    delete _stereo_thread;
    delete _read_thread;
    delete _visualize_thread;
    delete _settings;
    delete _stereo_image_io;
    delete _oxts_io;
}

//==============================================================================//

std::string MainDialog::createNewOutputDirectory()
{
    char buffer[1024];
    int system_status = 0;

    for (int32_t i=0; i<9999; i++)
    {
        sprintf(buffer,"output_%04d/I1_000000.png",i);
        FILE *file = fopen (buffer,"r");
        if (file!=NULL)
        {
            fclose (file);
            continue;
        }
        sprintf(buffer,"output_%04d/",i);
        break;
    }
    std::cout << "Creating output directory: " << buffer << std::endl;
    char cmd[1024];
    sprintf(cmd,"mkdir %s",buffer);

    // Do nothing. Just for resolving the warning message.
    system_status = system(cmd);
    if( system_status ) {};

    return buffer;
}

//==============================================================================//

void MainDialog::keyPressEvent(QKeyEvent * event)
{
    if (event->key()==Qt::Key_S)
    {
        _save_single_frame = false;
        on_stereoScanButton_clicked();
    }
    if (event->key()==Qt::Key_F)
    {
        _save_single_frame = true;
        on_stereoScanButton_clicked();
    }
}

//==============================================================================//

void MainDialog::on_shutterSpinBox_valueChanged(int )
{
    _settings->setValue("shutter_value",_ui->shutterSpinBox->value());
}

//==============================================================================//

void MainDialog::on_resizeSmallPushButton_clicked()
{
    int minw = _ui->modelView->minimumWidth();
    int minh = _ui->modelView->minimumHeight();
    int maxw = _ui->modelView->maximumWidth();
    int maxh = _ui->modelView->maximumHeight();

    _ui->modelView->setMinimumWidth(320);
    _ui->modelView->setMinimumHeight(480);
    _ui->modelView->setMaximumWidth(320);
    _ui->modelView->setMaximumHeight(480);
    _ui->modelView->setShowCamerasFlag(0);
    _ui->modelView->setGridFlag(0);

    _ui->modelView->recordHuman();

    _ui->modelView->setMinimumWidth (minw);
    _ui->modelView->setMinimumHeight(minh);
    _ui->modelView->setMaximumWidth (maxw);
    _ui->modelView->setMaximumHeight(maxh);
    _ui->modelView->setShowCamerasFlag(_ui->showCamerasCheckBox->isChecked());
    _ui->modelView->setGridFlag(_ui->gridCheckBox->isChecked());
}

//==============================================================================//

void MainDialog::on_whiteCheckBox_clicked()
{
    _ui->modelView->setWhiteFlag(_ui->whiteCheckBox->isChecked());
}

//==============================================================================//

void MainDialog::on_gridCheckBox_clicked()
{
    _ui->modelView->setGridFlag(_ui->gridCheckBox->isChecked());
}

//==============================================================================//

void MainDialog::on_recordPosesPushButton_clicked()
{
    _ui->modelView->playPoses(true);
}

//==============================================================================//

void MainDialog::on_playPosesPushButton_clicked()
{
    _ui->modelView->playPoses(false);
}

//==============================================================================//

void MainDialog::on_deletePosePushButton_clicked()
{
    _ui->modelView->delPose();
}

//==============================================================================//

void MainDialog::on_addPosePushButton_clicked()
{
    _ui->modelView->addPose();
}

//==============================================================================//

void MainDialog::on_resizePushButton_clicked()
{
    _ui->modelView->setMinimumWidth(800);
    _ui->modelView->setMinimumHeight(450);
}

//==============================================================================//

void MainDialog::on_showCamerasCheckBox_clicked()
{
    _ui->modelView->setShowCamerasFlag(_ui->showCamerasCheckBox->isChecked());
}

//==============================================================================//

void MainDialog::on_backgroundWallCheckBox_clicked()
{
    _ui->modelView->setBackgroundWallFlag(_ui->backgroundWallCheckBox->isChecked());
}

//==============================================================================//

void MainDialog::on_backgroundWallSlider_sliderMoved(int position)
{
    _ui->modelView->setBackgroundWallPosition((float)position/100.0);
}

//==============================================================================//

void MainDialog::on_resetBusButton_clicked()
{
    //cam_left->resetBus();
    //cam_right->resetBus();

    // show successfull reset message:
    std::cout << std::endl;
    std::cout << "=============================================" << std::endl;
    std::cout << "The firewire bus has been reset successfully!" << std::endl;
    std::cout << "=============================================" << std::endl;
}

//==============================================================================//

void MainDialog::on_readFromFilesCheckBox_clicked()
{
    if (_ui->readFromFilesCheckBox->isChecked())
    {
        _frame_index = 0;
    }
}

//==============================================================================//

void MainDialog::on_stereoScanButton_clicked()
{
    // stopping ... (wait for termination of visual odometry and stereo thread)
    if (_stereo_scan)
    {
        // set button text
        _ui->stereoScanButton->setText("Scan!");

        // stop stereo scanning
        _stereo_scan = false;

        // terminate all processes
        _visualOdomThread->terminate();
        _stereo_thread->terminate();
        _read_thread->terminate();
        while (_visualOdomThread->isRunning() || _stereo_thread->isRunning() || _read_thread->isRunning());

    // starting ...
    }
    else
    {
        // set button text
        _ui->stereoScanButton->setText("Stop!");

        // reset everything
        _visualOdomThread->resetHomographyTotal();
        _frame_index = 0;
        _gain_total   = 1;
        _ui->modelView->clearAll();
        _stereo_thread->clearReconstruction();

        // create output dir
        if (_ui->saveToFilesCheckBox->isChecked())
        {
            _output_dir = createNewOutputDirectory();
        }

        // start reading from files
        if (_ui->readFromFilesCheckBox->isChecked())
        {
            if (_read_thread->isRunning())
            {
                _read_thread->terminate();
            }
            QString input_dir = QFileDialog::getExistingDirectory (this,tr("Open Directory"),_settings->value("input_dir_name","/home/geiger").toString(),QFileDialog::ShowDirsOnly);
            _settings->setValue("input_dir_name",input_dir);
            _read_thread->setInputDir(input_dir);
            _read_thread->start();
        }

        // start stereo scanning
        _stereo_scan = true;
    }
}

//==============================================================================//

void MainDialog::on_exitButton_clicked()
{
    exit(0);
}

//==============================================================================//

void MainDialog::on_stopCapturingButton_clicked()
{
    //cam_left->stopRecording();
    //cam_right->stopRecording();
    _ui->captureFromFirewireButton->setEnabled(true);
    _ui->stopCapturingButton->setEnabled(false);
}

//==============================================================================//

void MainDialog::on_captureFromFirewireButton_clicked()
{
    /*
    SelectCamerasDialog dlg(cam_left,cam_right,_ui->shutterSpinBox->value(),_calib);
    dlg.exec();
    if (cam_left->isRunning()||cam_right->isRunning())
    {
        _ui->captureFromFirewireButton->setEnabled(false);
        _ui->stopCapturingButton->setEnabled(true);
    }
    */
}

//==============================================================================//

void MainDialog::newStereoImageArrived()
{
#if MAIN_DIALOG_DEBUG
    std::cout << "MainDialog::newStereoImageArrived" << std::endl;
#endif

    // get stereo image deepcopy
    StereoImage::simage simg(*_stereo_image->getStereoImage());
    _stereo_image->pickedUp();

    // save images
    if (_stereo_scan && (_ui->saveToFilesCheckBox->isChecked()))
    {
        if (_frame_index!=0)
        {
            std::cout << _stereo_image->timeDiff(simg.time,_last_frame_time) << std::endl;
        }
        _last_frame_time = simg.time;
        SaveStereoImageThread* save_thread = new SaveStereoImageThread(simg,_output_dir,_frame_index);
        save_thread->start();
        _save_stereo_threads.push_back(save_thread);

        // stop capturing if we requested only a single frame!
        if (_save_single_frame)
        {
            on_stereoScanButton_clicked();
        }
    }

    // check if any of the save threads have finished and must be deleted
    std::vector<SaveStereoImageThread*> running_save_stereo_threads;
    for (std::vector<SaveStereoImageThread*>::iterator it=_save_stereo_threads.begin(); it!=_save_stereo_threads.end(); it++)
    {
        if ((*it)->isRunning())
        {
            running_save_stereo_threads.push_back(*it);
        }
        else
        {
            delete *it;
        }
    }
    _save_stereo_threads = running_save_stereo_threads;

    // reconstruction mode
    if (_ui->tabWidget->currentIndex()==0)
    {
        // show image
        if (!_stereo_scan)
        {
            _ui->leftImageView->setImage(simg.I1,simg.width,simg.height);
            _ui->rightImageView->setImage(simg.I2,simg.width,simg.height);
        }

        // start quad matching if idle
        if (_stereo_scan && simg.rectified && !_visualOdomThread->isRunning())
        {
            QPalette p(palette());

            if (_ui->saveToFilesCheckBox->isChecked())
            {
                p.setColor(QPalette::Background, Qt::red);
            }
            else
            {
                p.setColor(QPalette::Background, Qt::green);
            }

            setPalette(p);
            _visualOdomThread->pushBack(simg,_ui->recordRawOdometryCheckBox->isChecked());
            _visualOdomThread->start();
        }
    }
    // elas mode
    else if (_ui->tabWidget->currentIndex()==1)
    {
        _ui->elasImageView->setImage(simg.I1,simg.width,simg.height);
        if (_stereo_scan && !_stereo_thread->isRunning() && !_ui->saveToFilesCheckBox->isChecked())
        {
            _stereo_thread->pushBack(simg,_ui->subsamplingCheckBox->isChecked());
            _stereo_thread->start();
        }
    }
    // left full image
    else if (_ui->tabWidget->currentIndex()==2)
    {
        _ui->leftFullImageView->setImage(simg.I1,simg.width,simg.height);
    }
    // right full image
    else if (_ui->tabWidget->currentIndex()==3)
    {
        _ui->rightFullImageView->setImage(simg.I2,simg.width,simg.height);
    }

    // Update frame index.
    ++_frame_index;
}

//==============================================================================//

void MainDialog::newHomographyArrived()
{
#if MAIN_DIALOG_DEBUG
    std::cout << "MainDialog::newHomographyArrived" << std::endl;
#endif

    // get stereo image deepcopy
    StereoImage::simage simg(*_visualOdomThread->getStereoImage());
    Matrix H_total = _visualOdomThread->getHomographyTotal();
    _gain_total *= _visualOdomThread->getGain();
    _visualOdomThread->pickedUp();

#if MAIN_DIALOG_DEBUG
    // Output the homography.
    std::cout << "Frame index: " << _frame_index << " || "
              << "Time stamp: " << ( simg.time.tv_sec * 1000 + simg.time.tv_usec ) << std::endl;
    std::cout << H_total << std::endl;
#endif

    // show quad match
    _ui->leftImageView->setImage(simg.I1,simg.width,simg.height);
    _ui->rightImageView->setImage(simg.I2,simg.width,simg.height);

    // show images
    _ui->leftImageView->setMatches(_visualOdomThread->getMatches(),_visualOdomThread->getInliers(),true);
    _ui->rightImageView->setMatches(_visualOdomThread->getMatches(),_visualOdomThread->getInliers(),false);

    // start dense stereo matching if idle
    if (_stereo_scan && !_stereo_thread->isRunning() && !_ui->saveToFilesCheckBox->isChecked())
    {
        _stereo_thread->pushBack(simg,H_total,_gain_total);
        _stereo_thread->start();
        _gain_total = 1;
        //_ui->modelView->addCamera(H_total,0.08,true);
    }
    else
    {
        //_ui->modelView->addCamera(H_total,0.05,false);
    }
}

//==============================================================================//

void MainDialog::newDisparityMapArrived()
{
#if MAIN_DIALOG_DEBUG
    std::cout << "MainDialog::newDisparityMapArrived" << std::endl;
#endif

    // get stereo image deepcopy
    StereoImage::simage simg(*_stereo_thread->getStereoImage());

    if (_ui->tabWidget->currentIndex()==0)
    {
        _ui->disparityView->setColorImage(_stereo_thread->getColorDisparityMap(),simg.width,simg.height);
    }
    else
    {
        if (!_ui->subsamplingCheckBox->isChecked())
        {
            _ui->elasDisparityView->setColorImage(_stereo_thread->getColorDisparityMap(),simg.width,simg.height);
        }
      else
        {
            _ui->elasDisparityView->setColorImage(_stereo_thread->getColorDisparityMap(),simg.width/2,simg.height/2);
        }
    }

    if (_stereo_scan && _ui->tabWidget->currentIndex()==0)
    {
        _ui->modelView->addCamera(_stereo_thread->getHomographyTotal(),0.1,true);
        _ui->modelView->addPoints(_stereo_thread->getPoints());
    }

    _stereo_thread->pickedUp();
}

//==============================================================================//

void MainDialog::onNewCalibrationData()
{
    std::cout << "on new _calibration data";
    std::cout << std::endl;
    _calib->pickedUp();
    _calib->showCalibrationParameters();

    if( _visualOdomThread->isRunning() )
    {
        _visualOdomThread->quit();
        while( _visualOdomThread->isRunning() );
    }
    //_visualOdomThread->disconnect();
    QObject::disconnect(_visualOdomThread,SIGNAL(newHomographyArrived()),this,SLOT(newHomographyArrived()));
    delete _visualOdomThread;
    _visualOdomThread = 0;

    if( _stereo_thread->isRunning() )
    {
        _stereo_thread->quit();
        while( _stereo_thread->isRunning() );
    }
    //_stereo_thread->disconnect();
    QObject::disconnect(_stereo_thread,SIGNAL(newDisparityMapArrived()),this,SLOT(newDisparityMapArrived()));
    delete _stereo_thread;
    _stereo_thread = 0;

    // Generate the new threads which need the _calibration data.
    //cam_left      = new FrameCaptureThread(_stereo_image,_calib,true,_capture_mutex);
    //cam_right     = new FrameCaptureThread(_stereo_image,_calib,false,_capture_mutex);
    _visualOdomThread     = new VisualOdometryThread(_calib);
    _stereo_thread = new StereoThread(_calib,_ui->modelView);

    // connect to the objects for communication.    
    //QObject::connect(_stereo_image,SIGNAL(newStereoImageArrived()),this,SLOT(newStereoImageArrived()));
    QObject::connect(_visualOdomThread,SIGNAL(newHomographyArrived()),this,SLOT(newHomographyArrived()));
    QObject::connect(_stereo_thread,SIGNAL(newDisparityMapArrived()),this,SLOT(newDisparityMapArrived()));
    //QObject::connect(_calib, SIGNAL( newCalibrationData() ), this, SLOT( onNewCalibrationData() ) );
}
