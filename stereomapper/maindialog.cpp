#include <iostream>
#include <unistd.h>

#include "maindialog.h"
#include "ui_maindialog.h"
#include "../libviso2/src/timer.h"

#define DEBUG_MAIN_DIALOG_ON_FUNCTION 0
#define DEBUG_SHOW_CALIBRATION_MATRIX 0
#define DEBUG_SHOW_VISUAL_ODOM_RESULT 0
#define DEBUG_SHOW_GNSS_INS_RESULT 0

#if DEBUG_SHOW_VISUAL_ODOM_RESULT
    int VISUAL_ODOM_COUNT = 0;
#endif

#if DEBUG_SHOW_GNSS_INS_RESULT
    int GNSS_INS_COUNT = 0;
#endif
MainDialog::MainDialog(QWidget *parent) :
    QDialog(parent),
    _ui(new Ui::MainDialog),
    _is_cam_to_imu_transformation_ready(false),
    _is_first_gnss_ins_ready(false)
{
    setWindowFlags(Qt::WindowMinMaxButtonsHint);
    _ui->setupUi(this);
    _stereo_image = new StereoImage();
    _gps_inertial_data = new GPSInertialData();
    _time_of_last_frame.tv_sec  = 0;
    _time_of_last_frame.tv_usec = 0;
    _calib = new CalibIOKITTI();
    _capture_mutex = new QMutex();
    _visual_odom_thread     = new VisualOdometryThread(_calib);
    _heading_filter = new HeadingFilter();
    _position_filter = new PositionFilter();
    _stereo_thread = new StereoThread(_calib,_ui->modelView);
    _stereo_image_io = new StereoImageIOKITTI();
    _gps_inertial_data_io = new GPSInertialDataIOKITTI();
    _read_thread   = new ReadFromFilesThread(_stereo_image, _gps_inertial_data, _calib, _stereo_image_io, _gps_inertial_data_io);
    _visualize_thread = new VisualizeThread(_ui->disparityView,_ui->modelView);

    // connect to the objects for communication.
    QObject::connect(_calib, SIGNAL(newCalibrationData()), this, SLOT( onNewCalibrationData() ) );
    QObject::connect(_stereo_image,SIGNAL(newStereoImageArrived()), this, SLOT(onNewStereoImageArrived()));
    QObject::connect(_gps_inertial_data, SIGNAL(newGPSInertialDataArrived()), this, SLOT(onNewGPSInertialDataArrived()));
    QObject::connect(_read_thread, SIGNAL(playbackDataFinished()), this, SLOT(onPlaybackDataFinished()));

    _frame_index = 0;
    _gain_total   = 1.0F;
    _stereo_scan  = false;
    _settings = new QSettings("KIT", "stereomapper");
    QPalette p(palette());
    p.setColor(QPalette::Background, Qt::white);
    setPalette(p);

    // Create GPX generator.
    _ground_truth_gpx_generator = new GpxGenerator();
    _visual_odom_gpx_generator = new GpxGenerator();

    _ui->modelView->setBackgroundWallFlag(_ui->backgroundWallCheckBox->isChecked());
    _ui->modelView->setShowCamerasFlag(_ui->showCamerasCheckBox->isChecked());
    _ui->modelView->setGridFlag(_ui->gridCheckBox->isChecked());
    _ui->modelView->setWhiteFlag(_ui->whiteCheckBox->isChecked());
}

//==============================================================================//

MainDialog::~MainDialog()
{
    delete _ui;
    delete _capture_mutex;
    delete _stereo_image;
    delete _gps_inertial_data;
    delete _calib;
    delete _visual_odom_gpx_generator;
    delete _stereo_thread;
    delete _read_thread;
    delete _visualize_thread;
    delete _settings;
    delete _stereo_image_io;
    delete _gps_inertial_data_io;

    if((_ground_truth_gpx_generator != NULL) &&
       (_ground_truth_gpx_generator->is_open()))
    {
        _ground_truth_gpx_generator->close();
        delete _ground_truth_gpx_generator;
    }

    if((_visual_odom_gpx_generator != NULL) &&
       (_visual_odom_gpx_generator->is_open()))
    {
        _visual_odom_gpx_generator->close();
        delete _visual_odom_gpx_generator;
    }

    if( _heading_filter != NULL )
    {
        delete _heading_filter;
    }

    if( _position_filter != NULL )
    {
        delete _position_filter;
    }
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

void MainDialog::on_resizeSmallPushButton_clicked()
{
#if DEBUG_MAIN_DIALOG_ON_FUNCTION
    std::cout << "MainDialog::on_resizeSmallPushButton_clicked" << std::endl;
#endif

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
#if DEBUG_MAIN_DIALOG_ON_FUNCTION
    std::cout << "MainDialog::on_whiteCheckBox_clicked" << std::endl;
#endif

    _ui->modelView->setWhiteFlag(_ui->whiteCheckBox->isChecked());
}

//==============================================================================//

void MainDialog::on_gridCheckBox_clicked()
{
#if DEBUG_MAIN_DIALOG_ON_FUNCTION
    std::cout << "MainDialog::on_gridCheckBox_clicked" << std::endl;
#endif

    _ui->modelView->setGridFlag(_ui->gridCheckBox->isChecked());
}

//==============================================================================//

void MainDialog::on_recordPosesPushButton_clicked()
{
#if DEBUG_MAIN_DIALOG_ON_FUNCTION
    std::cout << "MainDialog::on_recordPosesPushButton_clicked" << std::endl;
#endif

    _ui->modelView->playPoses(true);
}

//==============================================================================//

void MainDialog::on_playPosesPushButton_clicked()
{
#if DEBUG_MAIN_DIALOG_ON_FUNCTION
    std::cout << "MainDialog::on_playPosesPushButton_clicked" << std::endl;
#endif

    _ui->modelView->playPoses(false);
}

//==============================================================================//

void MainDialog::on_deletePosePushButton_clicked()
{
#if DEBUG_MAIN_DIALOG_ON_FUNCTION
    std::cout << "MainDialog::on_deletePosePushButton_clicked" << std::endl;
#endif

    _ui->modelView->delPose();
}

//==============================================================================//

void MainDialog::on_addPosePushButton_clicked()
{
#if DEBUG_MAIN_DIALOG_ON_FUNCTION
    std::cout << "MainDialog::on_addPosePushButton_clicked" << std::endl;
#endif

    _ui->modelView->addPose();
}

//==============================================================================//

void MainDialog::on_resizePushButton_clicked()
{
#if DEBUG_MAIN_DIALOG_ON_FUNCTION
    std::cout << "MainDialog::on_resizePushButton_clicked" << std::endl;
#endif

    _ui->modelView->setMinimumWidth(800);
    _ui->modelView->setMinimumHeight(450);
}

//==============================================================================//

void MainDialog::on_showCamerasCheckBox_clicked()
{
#if DEBUG_MAIN_DIALOG_ON_FUNCTION
    std::cout << "MainDialog::on_showCamerasCheckBox_clicked" << std::endl;
#endif

    _ui->modelView->setShowCamerasFlag(_ui->showCamerasCheckBox->isChecked());
}

//==============================================================================//

void MainDialog::on_backgroundWallCheckBox_clicked()
{
#if DEBUG_MAIN_DIALOG_ON_FUNCTION
    std::cout << "MainDialog::on_backgroundWallCheckBox_clicked" << std::endl;
#endif

    _ui->modelView->setBackgroundWallFlag(_ui->backgroundWallCheckBox->isChecked());
}

//==============================================================================//

void MainDialog::on_backgroundWallSlider_sliderMoved(int position)
{
#if DEBUG_MAIN_DIALOG_ON_FUNCTION
    std::cout << "MainDialog::on_backgroundWallSlider_sliderMoved" << std::endl;
#endif

    _ui->modelView->setBackgroundWallPosition((float)position/100.0);
}

//==============================================================================//

void MainDialog::on_stereoScanButton_clicked()
{
#if DEBUG_MAIN_DIALOG_ON_FUNCTION
    std::cout << "MainDialog::on_stereoScanButton_clicked" << std::endl;
#endif

    // stopping ... (wait for termination of visual odometry and stereo thread)
    if (_stereo_scan)
    {
        // set button text
        _ui->stereoScanButton->setText("Scan");

        // stop stereo scanning
        _stereo_scan = false;

        // reset the frame index.
        _frame_index = 0;

        // terminate all processes
        _visual_odom_thread->terminate();
        _stereo_thread->terminate();
        _read_thread->terminate();
        while (_visual_odom_thread->isRunning() || _stereo_thread->isRunning() || _read_thread->isRunning());

    // starting ...
    }
    else
    {
        // set button text
        _ui->stereoScanButton->setText("Stop");

        // reset everything
        _visual_odom_thread->resetHomographyTotal();
        _frame_index = 0;
        _gain_total   = 1;
        _ui->modelView->clearAll();
        _stereo_thread->clearReconstruction();

        // start reading from files
        if (_read_thread->isRunning())
        {
            _read_thread->terminate();
        }
        QString input_dir = QFileDialog::getExistingDirectory (this,tr("Open Directory"),_settings->value("input_dir_name","/").toString(),QFileDialog::ShowDirsOnly);
        _settings->setValue("input_dir_name",input_dir);
        _read_thread->setInputDir(input_dir);
        _read_thread->start();

        // open GPX file.
        QStringList path_list = input_dir.split('/');
        std::string gpx_file_name = path_list.last().toStdString() + "ground-truth.gpx";
        std::string gpx_full_file_path = input_dir.toStdString() + "/" + gpx_file_name;

        if(_ground_truth_gpx_generator->is_open())
        {
            _ground_truth_gpx_generator->close();
            _ground_truth_gpx_generator->reset();
        }
        _ground_truth_gpx_generator->open(gpx_full_file_path);

        gpx_file_name = path_list.last().toStdString() + "visual-odom.gpx";
        gpx_full_file_path = input_dir.toStdString() + "/" + gpx_file_name;

        if(_visual_odom_gpx_generator->is_open())
        {
            _visual_odom_gpx_generator->close();
            _visual_odom_gpx_generator->reset();
        }
        _visual_odom_gpx_generator->open(gpx_full_file_path);

        // start stereo scanning
        _stereo_scan = true;
    }

    _is_cam_to_imu_transformation_ready = false;
    _is_first_gnss_ins_ready = false;
}

//==============================================================================//

void MainDialog::on_exitButton_clicked()
{
    std::cout << "MainDialog::on_exitButton_clicked()" << std::endl;

    exit(0);
}

//==============================================================================//

void MainDialog::onNewCalibrationData()
{
#if DEBUG_MAIN_DIALOG_ON_FUNCTION
    std::cout << "MainDialog::onNewCalibrationData" << std::endl;
#endif
    _calib->pickedUp();

#if DEBUG_SHOW_CALIBRATION_MATRIX
    _calib->showCalibrationParameters();
#endif

    if( _visual_odom_thread->isRunning() )
    {
        _visual_odom_thread->quit();
        while( _visual_odom_thread->isRunning() );
    }

    QObject::disconnect(_visual_odom_thread,SIGNAL(newHomographyArrived()),this,SLOT(onNewHomographyArrived()));
    delete _visual_odom_thread;
    _visual_odom_thread = 0;

    if( _stereo_thread->isRunning() )
    {
        _stereo_thread->quit();
        while( _stereo_thread->isRunning() );
    }

    QObject::disconnect(_stereo_thread,SIGNAL(newDisparityMapArrived()),this,SLOT(onNewDisparityMapArrived()));
    delete _stereo_thread;
    _stereo_thread = 0;

    _visual_odom_thread     = new VisualOdometryThread(_calib);
    _stereo_thread = new StereoThread(_calib,_ui->modelView);

    // connect to the objects for communication.
    QObject::connect(_visual_odom_thread,SIGNAL(newHomographyArrived()),this,SLOT(onNewHomographyArrived()));
    QObject::connect(_stereo_thread,SIGNAL(newDisparityMapArrived()),this,SLOT(onNewDisparityMapArrived()));


    // Construct the transformation from camera coordinate to IMU coordinate.
    Matrix velo_to_cam_trans(4,4);
    velo_to_cam_trans.setDiag(1.0F);
    velo_to_cam_trans.setMat(_calib->_velo_to_cam_R, 0, 0);
    velo_to_cam_trans.setMat(Matrix::reshape(_calib->_velo_to_cam_T, 3, 1), 0, 3);

    Matrix imu_to_velo_trans(4,4);
    imu_to_velo_trans.setDiag(1.0F);
    imu_to_velo_trans.setMat(_calib->_imu_to_velo_R, 0, 0);
    imu_to_velo_trans.setMat(Matrix::reshape(_calib->_imu_to_velo_T, 3, 1), 0, 3);

    Matrix cam_to_velo_trans = Matrix::eye(4);
    Matrix velo_to_imu_trans = Matrix::eye(4);

    if((cam_to_velo_trans.solve(velo_to_cam_trans)) &&
       (velo_to_imu_trans.solve(imu_to_velo_trans)))
    {
        _cam_to_imu_trans = velo_to_imu_trans * cam_to_velo_trans;
        _is_cam_to_imu_transformation_ready = true;
    }
}

//==============================================================================//

void MainDialog::onNewStereoImageArrived()
{
#if DEBUG_MAIN_DIALOG_ON_FUNCTION
    std::cout << "MainDialog::onNewStereoImageArrived" << std::endl;
#endif

    // get stereo image deepcopy
    StereoImage::simage simg(*_stereo_image->getStereoImage());

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
        if (_stereo_scan && simg.rectified && !_visual_odom_thread->isRunning())
        {
            QPalette p(palette());

            p.setColor(QPalette::Background, Qt::green);

            setPalette(p);
            _visual_odom_thread->pushBack(simg, false); // TODO: let the user decide to record the raw odom or not.
            _visual_odom_thread->start();
        }
    }
    // elas mode
    else if (_ui->tabWidget->currentIndex()==1)
    {
        _ui->elasImageView->setImage(simg.I1,simg.width,simg.height);
        if (_stereo_scan && !_stereo_thread->isRunning())
        {
            _stereo_thread->pushBack(simg, _ui->subsamplingCheckBox->isChecked());
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

void MainDialog::onNewHomographyArrived()
{
#if DEBUG_MAIN_DIALOG_ON_FUNCTION
    std::cout << "MainDialog::newHomographyArrived" << std::endl;
#endif

    // get stereo image deepcopy
    StereoImage::simage simg(*_visual_odom_thread->getStereoImage());
    Matrix H_total = _visual_odom_thread->getHomographyTotal();
    _gain_total *= _visual_odom_thread->getGain();

    // show quad match
    _ui->leftImageView->setImage(simg.I1,simg.width,simg.height);
    _ui->rightImageView->setImage(simg.I2,simg.width,simg.height);

    // show images45349
    _ui->leftImageView->setMatches(_visual_odom_thread->getMatches(),_visual_odom_thread->getInliers(),true);
    _ui->rightImageView->setMatches(_visual_odom_thread->getMatches(),_visual_odom_thread->getInliers(),false);

    // start dense stereo matching if idle
    if (_stereo_scan && !_stereo_thread->isRunning())
    {
        _stereo_thread->pushBack(simg, H_total, _gain_total);
        _stereo_thread->start();
        _gain_total = 1;
        //_ui->modelView->addCamera(H_total,0.08,true);
    }
    else
    {
        //_ui->modelView->addCamera(H_total,0.05,false);
    }

    if( _is_cam_to_imu_transformation_ready )
    {
        // Calculate the delta_roll, delta_pitch, delta_yaw
        double delta_roll;
        double delta_pitch;
        double delta_yaw;
        double delta_velocity;
        double delta_altitude;
        _visual_odom_thread->getVisualOdomStatus( delta_roll, delta_pitch, delta_yaw,
                                                  delta_velocity, delta_altitude );
        _heading_filter->UpdateHeading( delta_yaw );

        // Calculate the delta_north_mov, delta_east_mov, delta_down_mov
        double cur_heading = _heading_filter->GetHeading();
        Coordinate::LOCAL_NED delta_ned(0, 0, 0);
        delta_ned._north = delta_velocity * sin( cur_heading );
        delta_ned._east = delta_velocity * cos( cur_heading );
        delta_ned._down = delta_altitude;

        // Update LLA position
        double lat_scale;
        double lon_scale;
        Coordinate::calculateScaleLLToNE( _position_filter->GetPosition(), lat_scale, lon_scale );
        Coordinate::GEOD_LLA delta_lla( delta_ned._north / lat_scale,
                                        delta_ned._east / lon_scale,
                                        delta_ned._down );
        _position_filter->UpdatePosition( delta_lla );
        Coordinate::GEOD_LLA cur_pos = _position_filter->GetPosition();

        // Write into the GPX file
        if( _visual_odom_gpx_generator->is_open() )
        {
            _visual_odom_gpx_generator->AddNewPosition( std::to_string( cur_pos._lat * Coordinate::R2D ),
                                                        std::to_string( cur_pos._lon * Coordinate::R2D ) );
        }

#if DEBUG_SHOW_VISUAL_ODOM_RESULT
        std::cout << "VISUAL_ODOM: " << cur_pos._lat * Coordinate::R2D <<
                     " , " << cur_pos._lon * Coordinate::R2D<< " , " << cur_heading << "\n====\n";
        VISUAL_ODOM_COUNT++;
#endif
    }
    _stereo_image->pickedUp(); // it is okay to retrieve next data
    _visual_odom_thread->pickedUp();
}

//==============================================================================//

void MainDialog::onNewDisparityMapArrived()
{
#if DEBUG_MAIN_DIALOG_ON_FUNCTION
    std::cout << "MainDialog::newDisparityMapArrived" << std::endl;
#endif

    // get stereo image deepcopy
    //StereoImage::simage simg(*_stereo_thread->getStereoImage());
    int simg_width = _stereo_thread->getStereoImage()->width;
    int simg_height = _stereo_thread->getStereoImage()->height;

    if (_ui->tabWidget->currentIndex()==0)
    {
        _ui->disparityView->setColorImage(_stereo_thread->getColorDisparityMap(), simg_width, simg_height);
    }
    else
    {
        if (!_ui->subsamplingCheckBox->isChecked())
        {
            _ui->elasDisparityView->setColorImage(_stereo_thread->getColorDisparityMap(), simg_width, simg_height);
        }
      else
        {
            _ui->elasDisparityView->setColorImage(_stereo_thread->getColorDisparityMap(), simg_width/2, simg_height/2);
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

void MainDialog::onNewGPSInertialDataArrived()
{
    // Fetch the data.
    GPSInertialDataFormat data = _gps_inertial_data->getData();

    // Store the first data.
    if( false == _is_first_gnss_ins_ready )
    {
        // Initialize the heading filter.
        _heading_filter->SetHeading( data.yaw );

        // Initialize the position filter.
        Coordinate::GEOD_LLA lla( data.lat * Coordinate::D2R,
                                  data.lon * Coordinate::D2R,
                                  data.alt );
        _position_filter->SetPosition( lla );

        _is_first_gnss_ins_ready = true;
    }

    // Write into the GPX file.
    if(_ground_truth_gpx_generator->is_open())
    {
        _ground_truth_gpx_generator->AddNewPosition(std::to_string(data.lat), std::to_string(data.lon));
    }
#if DEBUG_SHOW_GNSS_INS_RESULT
        std::cout << "GNSS_INS: " << data.lat << " , " << data.lon << " , " << data.yaw <<  "\n=====\n";
        GNSS_INS_COUNT++;
#endif
    _gps_inertial_data->pickedUp();
}

//==============================================================================//

void MainDialog::onPlaybackDataFinished()
{
    if(_ground_truth_gpx_generator->is_open())
    {
        _ground_truth_gpx_generator->close();
    }

    if(_visual_odom_gpx_generator->is_open())
    {
        _visual_odom_gpx_generator->close();
    }
}
