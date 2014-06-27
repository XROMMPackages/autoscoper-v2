#include "ui/AutoscoperMainWindow.h"
#include "ui_AutoscoperMainWindow.h"
#include <QtGui/QGridLayout>
#include "ui/FilterDockWidget.h"
#include "ui/VolumeDockWidget.h"
#include "ui/CameraViewWidget.h"
#include "ui/TimelineDockWidget.h"
#include "ui/TrackingOptionsDialog.h"
#include "ui/GLTracker.h"
#include "ui/ImportExportTrackingOptionsDialog.h"
#include "ui_ImportExportTrackingOptionsDialog.h"
#include "ui/OpenCLPlatformSelectDialog.h"
#include "Manip3D.hpp"

#include "ui/NewTrialDialog.h"
#include "ui_NewTrialDialog.h"

#include "Trial.hpp"
#include "View.hpp"
#include "Tracker.hpp"
#include "CoordFrame.hpp"

#include <QSplitter>
#include <QInputDialog>
#include <QList>
#include <QFileDialog>
#include <QGLContext>
#include <QMessageBox>
#include <QShortcut>



#include <iostream>
#include <fstream>
#include <sstream>

#ifdef WIN32
#define OS_SEP "\\"
#else
#define OS_SEP "/"
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

AutoscoperMainWindow::AutoscoperMainWindow(QWidget *parent) :
												QMainWindow(parent),
												ui(new Ui::AutoscoperMainWindow){
	
	//Setup UI
	ui->setupUi(this);

	//Init Tracker and get SharedGLContext
	tracker = new Tracker();
	gltracker = new GLTracker(tracker,NULL);
	shared_glcontext = gltracker->context();

	//History
	history = new History(10);
	first_undo = true;

	//Init empty trial
	trial_filename = "";
	is_trial_saved = true;
	is_tracking_saved = true;

	filters_widget =  new FilterDockWidget(this);
	this->addDockWidget(Qt::LeftDockWidgetArea, filters_widget);

	volumes_widget =  new VolumeDockWidget(this);
	this->addDockWidget(Qt::LeftDockWidgetArea, volumes_widget);

	timeline_widget =  new TimelineDockWidget(this);
	this->addDockWidget(Qt::BottomDockWidgetArea, timeline_widget);
	timeline_widget->setSharedGLContext(shared_glcontext);

	tracking_dialog = NULL;

	setupShortcuts();

#ifndef WITH_CUDA
	OpenCLPlatformSelectDialog * dialog = new OpenCLPlatformSelectDialog(this);
	if (dialog->getNumberPlatforms() > 1)dialog->exec();
	delete dialog;
#endif
	
}

AutoscoperMainWindow::~AutoscoperMainWindow(){
	delete ui;
	delete filters_widget;

	delete tracker;
	for (int i =0 ; i < manipulator.size(); i ++){
		delete manipulator[i];
	}
	manipulator.clear();

	delete history;
	if(tracking_dialog) {
		tracking_dialog->hide();
		delete tracking_dialog;
	}

	for (int i = 0 ; i < cameraViews.size();i++){
		delete cameraViews[i];
	}
	cameraViews.clear();
}

void AutoscoperMainWindow::closeEvent(QCloseEvent *event)
 {
	 save_trial_prompt();
     save_tracking_prompt();
     QMainWindow::closeEvent(event);
 }

//void AutoscoperMainWindow::setVolume_matrix(CoordFrame matrix){
//	delete volume_matrix;
//	volume_matrix = new CoordFrame(matrix);
//}

GraphData* AutoscoperMainWindow::getPosition_graph(){
	return timeline_widget->getPosition_graph();
}

Manip3D * AutoscoperMainWindow::getManipulator(int idx){
	if(idx < manipulator.size() && 
		idx >= 0){
		return manipulator[idx];
	}else if(getTracker()->trial()->num_frames > 0 ){
		return manipulator[getTracker()->trial()->current_volume];
	}else{
		return NULL;
	}			
}

void AutoscoperMainWindow::update_graph_min_max(int frame){
	timeline_widget->update_graph_min_max(frame);
}

void AutoscoperMainWindow::relayoutCameras(int rows){
	//clear Old Splitters
	for (int i = 0 ; i < cameraViews.size();i++){
		cameraViews[i]->setParent(NULL);
	}

	QObjectList objs = ui->frameWindows->children();
	for(int x = 0; x < objs.size(); x ++){
		QObjectList objs2 = objs[x]->children();
		QSplitter * vertsplit = dynamic_cast<QSplitter*> (objs[x]);
		for(int y = objs2.size() - 1 ; y >= 0; y --){	
			QSplitter * horsplit = dynamic_cast<QSplitter*> (objs2[y]);
			if(horsplit){
				QObjectList objs3 = horsplit->children();
				delete horsplit;
			}
		}	
		
		if(vertsplit){
			ui->gridLayout->removeWidget(vertsplit);
			delete vertsplit;
		}
	}
	//Create New
	cameraViewArrangement = QSize(ceil( ((double) cameraViews.size())/rows),rows);

	QSplitter * splitter = new QSplitter(this);
    splitter->setOrientation(Qt::Vertical);
	for(int i = 0; i < cameraViewArrangement.height() ; i ++){
		QSplitter * splitterHorizontal = new QSplitter(splitter);
		splitterHorizontal->setOrientation(Qt::Horizontal);
		splitter->addWidget(splitterHorizontal);
	}

	int freeSpaces = cameraViewArrangement.height() * cameraViewArrangement.width() - cameraViews.size();
	
	int count = 0;
	for (int i = 0 ; i < cameraViews.size();i++, count++){
		if(cameraViews.size() < i + freeSpaces) count++;
		QObject* obj = splitter->children().at(rows - 1 - count / (cameraViewArrangement.width()));
		QSplitter * horsplit = dynamic_cast<QSplitter*> (obj);
		if(horsplit)horsplit->addWidget(cameraViews[i]);
	}

	for(int i = 0; i < cameraViewArrangement.height() ; i ++){
		QObject* obj = splitter->children().at(i);
		QSplitter * horsplit = dynamic_cast<QSplitter*> (obj);
		if(horsplit){
			QList<int> sizelist = horsplit->sizes();
			for(int m = 0; m < sizelist.size(); m++){
					sizelist[m] = 1;
			}
			horsplit->setSizes(sizelist);
		}
	}


	ui->gridLayout->addWidget(splitter, 0, 0, 1, 1);
}

void AutoscoperMainWindow::timelineSetValue(int value){
	tracker->trial()->frame = value;
    frame_changed();
}

void AutoscoperMainWindow::frame_changed()
{
    // Lock or unlock the position
	if (timeline_widget->getPosition_graph()->frame_locks.at(tracker->trial()->frame)) {       
		timeline_widget->setValuesEnabled(false);
    }
    else {
       timeline_widget->setValuesEnabled(true);
    }

    update_xyzypr_and_coord_frame();

    for (unsigned int i = 0; i < tracker->trial()->cameras.size(); ++i) {
        tracker->trial()->videos.at(i).set_frame(tracker->trial()->frame);
        tracker->view(i)->radRenderer()->set_rad(
            tracker->trial()->videos.at(i).data(),
            tracker->trial()->videos.at(i).width(),
            tracker->trial()->videos.at(i).height(),
            tracker->trial()->videos.at(i).bps());

        glBindTexture(GL_TEXTURE_2D,textures[i]);
        glTexImage2D(GL_TEXTURE_2D,
                     0,
                     1,
                     tracker->trial()->videos.at(i).width(),
                     tracker->trial()->videos.at(i).height(),
                     0,
                     GL_LUMINANCE,
                    (tracker->trial()->videos.at(i).bps() == 8? GL_UNSIGNED_BYTE:
                                                               GL_UNSIGNED_SHORT),
                     tracker->trial()->videos.at(i).data());
        glBindTexture(GL_TEXTURE_2D,0);
    }

	redrawGL();
	QApplication::processEvents(QEventLoop::ExcludeUserInputEvents);
}


void AutoscoperMainWindow::volume_changed()
{
    // Lock or unlock the position
	if (timeline_widget->getPosition_graph()->frame_locks.at(tracker->trial()->frame)) {       
		timeline_widget->setValuesEnabled(false);
    }
    else {
       timeline_widget->setValuesEnabled(true);
    }

	getManipulator()->set_movePivot(ui->toolButtonMovePivot->isChecked());

    //update_xyzypr_and_coord_frame();
	timeline_widget->getSelectedNodes()->clear();
	update_graph_min_max(timeline_widget->getPosition_graph());

	redrawGL();
	QApplication::processEvents(QEventLoop::ExcludeUserInputEvents);
}

void AutoscoperMainWindow::update_xyzypr()
{
    double xyzypr[6];
   (CoordFrame::from_matrix(trans(getManipulator(-1)->transform())) * *tracker->trial()->getVolumeMatrix(-1)).to_xyzypr(xyzypr);

    ////Update the spin buttons.
    timeline_widget->setSpinButtonUpdate(false);
    timeline_widget->setValues(&xyzypr[0]);
	timeline_widget->setSpinButtonUpdate(true);
	redrawGL();
}

// Updates the coordinate frames position after the spin buttons values have
// been changed.
void AutoscoperMainWindow::update_xyzypr_and_coord_frame()
{
	for(int i = 0; i < tracker->trial()->num_volumes; i++){
		if (tracker->trial()->getXCurve(i)->empty()) {
			continue;
		}

		double xyzypr[6];
		xyzypr[0] = (*tracker->trial()->getXCurve(i))(tracker->trial()->frame);
		xyzypr[1] = (*tracker->trial()->getYCurve(i))(tracker->trial()->frame);
		xyzypr[2] = (*tracker->trial()->getZCurve(i))(tracker->trial()->frame);
		xyzypr[3] = (*tracker->trial()->getYawCurve(i))(tracker->trial()->frame);
		xyzypr[4] = (*tracker->trial()->getPitchCurve(i))(tracker->trial()->frame);
		xyzypr[5] = (*tracker->trial()->getRollCurve(i))(tracker->trial()->frame);

		CoordFrame newCoordFrame = CoordFrame::from_xyzypr(xyzypr);
		set_manip_matrix(i, newCoordFrame*tracker->trial()->getVolumeMatrix(i)->inverse());

		if(i == tracker->trial()->current_volume){
			timeline_widget->setSpinButtonUpdate(false);
			timeline_widget->setValues(&xyzypr[0]);
			timeline_widget->setSpinButtonUpdate(true);
		}
	}
}

void AutoscoperMainWindow::set_manip_matrix(int idx, const CoordFrame& frame)
{
    double m[16];
    frame.to_matrix_row_order(m);
    getManipulator(idx)->set_transform(Mat4d(m));
}

// Automatically updates the graph's minimum and maximum values to stretch the
// data the full height of the viewport.
void AutoscoperMainWindow::update_graph_min_max(GraphData* graph, int frame)
{
    if (!tracker->trial() || tracker->trial()->getXCurve(-1)->empty()) {
        graph->max_value = 180.0;
        graph->min_value = -180.0;
    }
    // If a frame is specified then only check that frame for a new minimum and
    // maximum.
    else if (frame != -1) {
        if (graph->show_x) {
            float x_value = (*tracker->trial()->getXCurve(-1))(frame);
            if (x_value > graph->max_value) {
                graph->max_value = x_value;
            }
            if (x_value < graph->min_value) {
                graph->min_value = x_value;
            }
        }
        if (graph->show_y) {
            float y_value = (*tracker->trial()->getYCurve(-1))(frame);
            if (y_value > graph->max_value) {
                graph->max_value = y_value;
            }
            if (y_value < graph->min_value) {
                graph->min_value = y_value;
            }
        }
        if (graph->show_z) {
            float z_value = (*tracker->trial()->getZCurve(-1))(frame);
            if (z_value > graph->max_value) {
                graph->max_value = z_value;
            }
            if (z_value < graph->min_value) {
                graph->min_value = z_value;
            }
        }
        if (graph->show_yaw) {
            float yaw_value = (*tracker->trial()->getYawCurve(-1))(frame);
            if (yaw_value > graph->max_value) {
                graph->max_value = yaw_value;
            }
            if (yaw_value < graph->min_value) {
                graph->min_value = yaw_value;
            }
        }
        if (graph->show_pitch) {
            float pitch_value = (*tracker->trial()->getPitchCurve(-1))(frame);
            if (pitch_value > graph->max_value) {
                graph->max_value = pitch_value;
            }
            if (pitch_value < graph->min_value) {
                graph->min_value = pitch_value;
            }
        }
        if (graph->show_roll) {
            float roll_value = (*tracker->trial()->getRollCurve(-1))(frame);
            if (roll_value > graph->max_value) {
                graph->max_value = roll_value;
            }
            if (roll_value < graph->min_value) {
                graph->min_value = roll_value;
            }
        }
    }
    // Otherwise we need to check all the frames.
    else {

        graph->min_value = 1e6;
        graph->max_value = -1e6;

        if (graph->show_x) {
            for (frame = floor(graph->min_frame);
                 frame < graph->max_frame;
                 frame += 1.0f) {
                float x_value = (*tracker->trial()->getXCurve(-1))(frame);
                if (x_value > graph->max_value) {
                    graph->max_value = x_value;
                }
                if (x_value < graph->min_value) {
                    graph->min_value = x_value;
                }
            }
        }
        if (graph->show_y) {
            for (frame = floor(graph->min_frame);
                 frame < graph->max_frame;
                 frame += 1.0f) {
                float y_value = (*tracker->trial()->getYCurve(-1))(frame);
                if (y_value > graph->max_value) {
                    graph->max_value = y_value;
                }
                if (y_value < graph->min_value) {
                    graph->min_value = y_value;
                }
            }
        }
        if (graph->show_z) {
            for (frame = floor(graph->min_frame);
                 frame < graph->max_frame;
                 frame += 1.0f) {
                float z_value = (*tracker->trial()->getZCurve(-1))(frame);
                if (z_value > graph->max_value) {
                    graph->max_value = z_value;
                }
                if (z_value < graph->min_value) {
                    graph->min_value = z_value;
                }
            }
        }
        if (graph->show_yaw) {
            for (frame = floor(graph->min_frame);
                 frame < graph->max_frame;
                 frame += 1.0f) {
                float yaw_value = (*tracker->trial()->getYawCurve(-1))(frame);
                if (yaw_value > graph->max_value) {
                    graph->max_value = yaw_value;
                }
                if (yaw_value < graph->min_value) {
                    graph->min_value = yaw_value;
                }
            }
        }
        if (graph->show_pitch) {
            for (frame = floor(graph->min_frame);
                 frame < graph->max_frame;
                 frame += 1.0f) {
                float pitch_value = (*tracker->trial()->getPitchCurve(-1))(frame);
                if (pitch_value > graph->max_value) {
                    graph->max_value = pitch_value;
                }
                if (pitch_value < graph->min_value) {
                    graph->min_value = pitch_value;
                }
            }
        }
        if (graph->show_roll) {
            for (frame = floor(graph->min_frame);
                 frame < graph->max_frame;
                 frame += 1.0f) {
                float roll_value = (*tracker->trial()->getRollCurve(-1))(frame);
                if (roll_value > graph->max_value) {
                    graph->max_value = roll_value;
                }
                if (roll_value < graph->min_value) {
                    graph->min_value = roll_value;
                }
            }
        }

        graph->min_value -= 1.0;
        graph->max_value += 1.0;
    }
}

void AutoscoperMainWindow::setupUI()
{
    //Remove previous cameras
    for (unsigned int i = 0; i < cameraViews.size(); i++) {
		cameraViews[i]->setParent(NULL);
		delete cameraViews[i];
    }
	cameraViews.clear();
	filters_widget->clearTree();
	volumes_widget->clear();

	//Add Volumes
	 for (unsigned int i = 0; i < tracker->trial()->volumes.size(); i++) {
		 volumes_widget->addVolume(tracker->trial()->volumes[i].name());
    }

    //Add the new cameras
    for (unsigned int i = 0; i < tracker->trial()->cameras.size(); i++) {
		cameraViews.push_back(new CameraViewWidget(i, tracker->view(i),tracker->trial()->cameras[i].mayacam().c_str(), this));
		cameraViews[i]->setSharedGLContext(shared_glcontext);	
		filters_widget->addCamera(tracker->view(i));
    }
	relayoutCameras(1);
    textures.resize(tracker->trial()->cameras.size());
    for (unsigned i = 0; i < textures.size(); i++) {
        glGenTextures(1,&textures[i]);
        glBindTexture(GL_TEXTURE_2D,textures[i]);
        glTexImage2D(GL_TEXTURE_2D,
                     0,
                     1,
                     tracker->trial()->videos.at(i).width(),
                     tracker->trial()->videos.at(i).height(),
                     0,
                     GL_LUMINANCE,
                    (tracker->trial()->videos.at(i).bps() == 8? GL_UNSIGNED_BYTE:
                                                               GL_UNSIGNED_SHORT),
                     tracker->trial()->videos.at(i).data());
        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP);
        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP);
        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
        glBindTexture(GL_TEXTURE_2D,0);
    }
	// Setup the default view

    // Update the number of frames
	timeline_widget->setFramesRange(0,tracker->trial()->num_frames-1);
    reset_graph();

    // Update the coordinate frames
	timeline_widget->getPosition_graph()->min_frame = 0;
	timeline_widget->getPosition_graph()->max_frame = tracker->trial()->num_frames-1; //
    timeline_widget->getPosition_graph()->frame_locks = vector<bool>(tracker->trial()->num_frames,false);

    update_graph_min_max(timeline_widget->getPosition_graph());

    frame_changed();
}

void AutoscoperMainWindow::redrawGL(){
	for (unsigned int i = 0; i < cameraViews.size(); i++) {
		cameraViews[i]->draw();
    }
	timeline_widget->draw();
}

void AutoscoperMainWindow::setFrame(int frame){
	tracker->trial()->frame = frame;
	timeline_widget->setFrame(frame);
    frame_changed();
}

void AutoscoperMainWindow::update_coord_frame()
{
    double xyzypr[6];
	timeline_widget->getValues(&xyzypr[0]);
	if(tracker->trial()) {
		CoordFrame newCoordFrame = CoordFrame::from_xyzypr(xyzypr);
		set_manip_matrix(tracker->trial()->current_volume, newCoordFrame*tracker->trial()->getVolumeMatrix(-1)->inverse());
	}
	redrawGL();
}


//Saving and Loading
void AutoscoperMainWindow::save_trial_prompt()
{
    if (is_trial_saved) {
        return;
    }

	QMessageBox::StandardButton reply;
	reply = QMessageBox::question(this, "", "Would you like to save the current trial?",
                                QMessageBox::Yes|QMessageBox::No);
	if (reply == QMessageBox::Yes) {
		//on_save_trial1_activate(NULL,NULL);
	}
}

void AutoscoperMainWindow::save_tracking_prompt()
{
    if (is_tracking_saved) {
        return;
    }

	QMessageBox::StandardButton reply;
	reply = QMessageBox::question(this, "", "Would you like to export the unsaved tracking data?",
                                QMessageBox::Yes|QMessageBox::No);
	if (reply == QMessageBox::Yes) {
		QString filename = get_filename(true, "*.tra");
		save_tracking_results(filename);
	}
}

QString AutoscoperMainWindow::get_filename(bool save, QString type)
{	
	QString FileName = "";
	if(save){
		FileName = QFileDialog::getSaveFileName(this,
									tr("Save File as"), QDir::currentPath(),tr("CFG Files (") + type + tr(" *.cfg)"));
	}else{
		FileName = QFileDialog::getOpenFileName(this,
									tr("Open File"), QDir::currentPath(),tr("CFG Files (") + type + tr(" *.cfg)"));
	}

	return FileName;
}

void AutoscoperMainWindow::save_tracking_results(QString filename)
{
	ImportExportTrackingOptionsDialog * diag = new ImportExportTrackingOptionsDialog(this);
	diag->exec();

	if(diag->result()){
		bool save_as_matrix = diag->diag->radioButton_TypeMatrix->isChecked();
		bool save_as_rows = diag->diag->radioButton_OrientationRow->isChecked();
		bool save_with_commas = diag->diag->radioButton_SeperatorComma->isChecked();
		bool convert_to_cm = diag->diag->radioButton_TranslationCM->isChecked();
		bool convert_to_rad = diag->diag->radioButton_RotationRadians->isChecked();
		bool interpolate = diag->diag->radioButton_InterpolationSpline->isChecked();

		const char* s = save_with_commas? "," :" ";

		std::ofstream file(filename.toStdString().c_str(), ios::out);

		file.precision(16);
		file.setf(ios::fixed,ios::floatfield);
		bool invalid;
		for (int i = 0; i < tracker->trial()->num_frames; ++i) {
			for(int j = 0; j < tracker->trial()->num_volumes ; j++ ){
				if (!interpolate) {
					if (tracker->trial()->getXCurve(-1)->find(i) ==
							tracker->trial()->getXCurve(-1)->end() &&
						tracker->trial()->getYCurve(-1)->find(i) ==
							tracker->trial()->getYCurve(-1)->end() &&
						tracker->trial()->getZCurve(-1)->find(i) ==
							tracker->trial()->getZCurve(-1)->end() &&
						tracker->trial()->getYawCurve(-1)->find(i) ==
							tracker->trial()->getYawCurve(-1)->end() &&
						tracker->trial()->getPitchCurve(-1)->find(i) ==
							tracker->trial()->getPitchCurve(-1)->end() &&
						tracker->trial()->getRollCurve(-1)->find(i) ==
							tracker->trial()->getRollCurve(-1)->end()) {
						invalid = true;			
					}else{
						invalid = false;
					}
				}else{
					invalid = false;
				}

				if(invalid){
					if (save_as_matrix) {
						file << "NaN";
						for (int j = 0; j < 15; j++) { file << s << "NaN"; }
					}
					else {
						file << "NaN";
						for (int j = 0; j < 5; j++) { file << s << "NaN"; }
					}
				}else{
					double xyzypr[6];
					xyzypr[0] = (*tracker->trial()->getXCurve(j))(i);
					xyzypr[1] = (*tracker->trial()->getYCurve(j))(i);
					xyzypr[2] = (*tracker->trial()->getZCurve(j))(i);
					xyzypr[3] = (*tracker->trial()->getYawCurve(j))(i);
					xyzypr[4] = (*tracker->trial()->getPitchCurve(j))(i);
					xyzypr[5] = (*tracker->trial()->getRollCurve(j))(i);

					if (save_as_matrix) {
						double m[16];
						CoordFrame::from_xyzypr(xyzypr).to_matrix(m);

						if (convert_to_cm) {
							m[12] /= 10.0;
							m[13] /= 10.0;
							m[14] /= 10.0;
						}

						if (save_as_rows) {
							file << m[0] << s << m[4] << s << m[8] << s << m[12] << s
								 << m[1] << s << m[5] << s << m[9] << s << m[13] << s
								 << m[2] << s << m[6] << s << m[10] << s << m[14] << s
								 << m[3] << s << m[7] << s << m[11] << s<< m[15];
						}
						else {
							file << m[0] << s << m[1] << s << m[2] << s << m[3] << s
								 << m[4] << s << m[5] << s << m[6] << s << m[7] << s
								 << m[8] << s << m[9] << s << m[10] << s << m[11] << s
								 << m[12] << s << m[13] << s << m[14] << s<< m[15];
							 
						}
					}
					else {
						if (convert_to_cm) {
							xyzypr[0] /= 10.0;
							xyzypr[1] /= 10.0;
							xyzypr[2] /= 10.0;
						}
						if (convert_to_rad) {
							xyzypr[3] *= M_PI/180.0;
							xyzypr[4] *= M_PI/180.0;
							xyzypr[5] *= M_PI/180.0;
						}

						file << xyzypr[0] << s << xyzypr[1] << s << xyzypr[2] << s
							 << xyzypr[3] << s << xyzypr[4] << s << xyzypr[5];
					}
				}

				if (j != tracker->trial()->num_volumes - 1) file << s ;
			}
			file << endl;
		}
		file.close();

		is_tracking_saved = true;
	}
	delete diag;
}

void AutoscoperMainWindow::load_tracking_results(QString filename)
{
    save_tracking_prompt();

	ImportExportTrackingOptionsDialog * diag = new ImportExportTrackingOptionsDialog(this);
	diag->exec();

	if(diag->result()){
		bool save_as_matrix = diag->diag->radioButton_TypeMatrix->isChecked();
		bool save_as_rows = diag->diag->radioButton_OrientationRow->isChecked();
		bool save_with_commas = diag->diag->radioButton_SeperatorComma->isChecked();
		bool convert_to_cm = diag->diag->radioButton_TranslationCM->isChecked();
		bool convert_to_rad = diag->diag->radioButton_RotationRadians->isChecked();
		//bool interpolate = diag->diag->radioButton_InterpolationSpline->isChecked();

		char s = save_with_commas? ',': ' ';

		std::ifstream file(filename.toStdString().c_str(), ios::in);

		for(int j = 0; j < tracker->trial()->num_volumes ; j++ ){
			tracker->trial()->getXCurve(j)->clear();
			tracker->trial()->getYCurve(j)->clear();
			tracker->trial()->getZCurve(j)->clear();
			tracker->trial()->getYawCurve(j)->clear();
			tracker->trial()->getPitchCurve(j)->clear();
			tracker->trial()->getRollCurve(j)->clear();
		}

		double m[16];
		string line, value;
		for (int i = 0; i < tracker->trial()->num_frames && getline(file,line); ++i) {
			istringstream lineStream(line);
			for (int k = 0; k < tracker->trial()->num_volumes ; k++ ){
				for (int j = 0; j < (save_as_matrix? 16: 6) && getline(lineStream, value, s); ++j) {
					istringstream valStream(value);
					valStream >> m[j];
				}

				if (value.compare(0,3,"NaN") == 0) {
					continue;
				}

				if (save_as_matrix && save_as_rows) {
					double n[16];
					memcpy(n,m,16*sizeof(double));
					m[1] = n[4];
					m[2] = n[8];
					m[3] = n[12];
					m[4] = n[1];
					m[6] = n[9];
					m[7] = n[13];
					m[8] = n[2];
					m[9] = n[6];
					m[11] = n[14];
					m[12] = n[3];
					m[13] = n[7];
					m[14] = n[11];
				}

				if (convert_to_cm) {
					if (save_as_matrix) {
						m[12] *= 10.0;
						m[13] *= 10.0;
						m[14] *= 10.0;
					}
					else {
						m[0] *= 10.0;
						m[1] *= 10.0;
						m[2] *= 10.0;
					}
				}

				if (convert_to_rad) {
					if (!save_as_matrix) {
						m[3] *= 180.0/M_PI;
						m[4] *= 180.0/M_PI;
						m[5] *= 180.0/M_PI;
					}
				}

				if (save_as_matrix) {
					CoordFrame::from_matrix(m).to_xyzypr(m);
				}

				tracker->trial()->getXCurve(k)->insert(i,m[0]);
				tracker->trial()->getYCurve(k)->insert(i,m[1]);
				tracker->trial()->getZCurve(k)->insert(i,m[2]);
				tracker->trial()->getYawCurve(k)->insert(i,m[3]);
				tracker->trial()->getPitchCurve(k)->insert(i,m[4]);
				tracker->trial()->getRollCurve(k)->insert(i,m[5]);
			}
		}
		file.close();

		is_tracking_saved = true;

		frame_changed();
		update_graph_min_max(timeline_widget->getPosition_graph());

		redrawGL();
	}
	delete diag;
}

void AutoscoperMainWindow::openTrial(){
	QString cfg_fileName = get_filename(false);

	if ( cfg_fileName.isNull() == false )
    {
        try {
			Trial * trial = new Trial(cfg_fileName.toStdString().c_str());
			tracker->load(*trial);
			delete trial;

			trial_filename = cfg_fileName.toStdString();
			is_trial_saved = true;
			is_tracking_saved = true;

			for (int i =0 ; i < manipulator.size(); i ++){
				delete manipulator[i];
			}
			manipulator.clear();
			for(int i = 0 ; i < tracker->trial()->num_volumes; i ++){
				manipulator.push_back(new Manip3D());
				getManipulator(i)->set_transform(Mat4d());
			}

			setupUI();
			timelineSetValue(0);

			timeline_widget->setTrial(tracker->trial());

		}
		catch (std::exception& e) {
			std::cerr << e.what() << std::endl;
		}
	}
}

void AutoscoperMainWindow::newTrial(){
	NewTrialDialog * diag = new NewTrialDialog(this);
	diag->exec();

	if(diag->result()){
		try {
			tracker->load(diag->trial);

			trial_filename = "";
			is_trial_saved = false;
			is_tracking_saved = true;


			for (int i =0 ; i < manipulator.size(); i ++){
				delete manipulator[i];
			}
			manipulator.clear();
			for(int i = 0 ; i < tracker->trial()->num_volumes; i ++){
				manipulator.push_back(new Manip3D());
				getManipulator(i)->set_transform(Mat4d());
			}
			
			setupUI();
			timelineSetValue(0);

			timeline_widget->setTrial(tracker->trial());

		}
		catch (std::exception& e) {
			std::cerr << e.what() << std::endl;
		}
	}
	delete diag;
}

//History
void AutoscoperMainWindow::push_state()
{
    State current_state;
	for(int i = 0 ; i < tracker->trial()->num_volumes; i++){
		current_state.x_curve.push_back(*tracker->trial()->getXCurve(i));
		current_state.y_curve.push_back(*tracker->trial()->getYCurve(i));
		current_state.z_curve.push_back(*tracker->trial()->getZCurve(i));
		current_state.x_rot_curve.push_back(*tracker->trial()->getYawCurve(i));
		current_state.y_rot_curve.push_back(*tracker->trial()->getPitchCurve(i));
		current_state.z_rot_curve.push_back(*tracker->trial()->getRollCurve(i));
	}

    history->push(current_state);

    first_undo = true;
    is_tracking_saved = false;
}

void AutoscoperMainWindow::undo_state()
{
    if (history->can_undo()) {

        if (first_undo) {
            push_state();
            history->undo();
            first_undo = false;
        }

        State undo_state = history->undo();

		for(int i = 0 ; i < tracker->trial()->num_volumes; i++){
			*tracker->trial()->getXCurve(i) = undo_state.x_curve[i];
			*tracker->trial()->getYCurve(i) = undo_state.y_curve[i];
			*tracker->trial()->getZCurve(i) = undo_state.z_curve[i];
			*tracker->trial()->getYawCurve(i) = undo_state.x_rot_curve[i];
			*tracker->trial()->getPitchCurve(i) = undo_state.y_rot_curve[i];
			*tracker->trial()->getRollCurve(i) = undo_state.z_rot_curve[i];
		}
        timeline_widget->getSelectedNodes()->clear();

        update_graph_min_max(timeline_widget->getPosition_graph());
        update_xyzypr_and_coord_frame();
        
		redrawGL();
    }
}

void AutoscoperMainWindow::redo_state()
{
    if (history->can_redo()) {
        State redo_state = history->redo();

		for(int i = 0 ; i < tracker->trial()->num_volumes; i++){
			*tracker->trial()->getXCurve(i) = redo_state.x_curve[i];
			*tracker->trial()->getYCurve(i) = redo_state.y_curve[i];
			*tracker->trial()->getZCurve(i) = redo_state.z_curve[i];
			*tracker->trial()->getYawCurve(i) = redo_state.x_rot_curve[i];
			*tracker->trial()->getPitchCurve(i) = redo_state.y_rot_curve[i];
			*tracker->trial()->getRollCurve(i) = redo_state.z_rot_curve[i];
		}
		timeline_widget->getSelectedNodes()->clear();

        update_graph_min_max(timeline_widget->getPosition_graph());
        update_xyzypr_and_coord_frame();
        
		redrawGL();
    }
}

void AutoscoperMainWindow::reset_graph()
{
	for(int i = 0 ; i < tracker->trial()->num_volumes; i++){
		tracker->trial()->getXCurve(i)->clear();
		tracker->trial()->getYCurve(i)->clear();
		tracker->trial()->getZCurve(i)->clear();
		tracker->trial()->getYawCurve(i)->clear();
		tracker->trial()->getPitchCurve(i)->clear();
		tracker->trial()->getRollCurve(i)->clear();
	}
	timeline_widget->getCopiedNodes()->clear();
}

//File menu

void AutoscoperMainWindow::on_actionNew_triggered(bool checked){
	save_trial_prompt();
    save_tracking_prompt();

	newTrial();
}
void AutoscoperMainWindow::on_actionOpen_triggered(bool checked){
	save_trial_prompt();
    save_tracking_prompt();
	
	openTrial();
}

void AutoscoperMainWindow::on_actionSave_triggered(bool checked){
	if (trial_filename.compare("") == 0) {
        on_actionSave_as_triggered(true);
    }
    else {
        try {
            tracker->trial()->save(trial_filename);
            is_trial_saved = true;
        }
        catch (exception& e) {
            cerr << e.what() << endl;
        }
    }
}

void AutoscoperMainWindow::on_actionSave_as_triggered(bool checked){
	QString filename = get_filename(true);
    if (filename.compare("") != 0) {
        try {
			trial_filename = filename.toStdString().c_str();
			tracker->trial()->save(trial_filename);
            is_tracking_saved = true;
        }
        catch (exception& e) {
            cerr << e.what() << endl;
        }
    }
}

void AutoscoperMainWindow::on_actionImport_Tracking_triggered(bool checked){
	QString filename = get_filename(false, "*.tra");
    if (filename.compare("") != 0) {
        load_tracking_results(filename);
    }
}

void AutoscoperMainWindow::on_actionExport_Tracking_triggered(bool checked){
	QString filename = get_filename(true, "*.tra");
    if (filename.compare("") != 0) {
        save_tracking_results(filename);
    }
}

void AutoscoperMainWindow::on_actionQuit_triggered(bool checked){
	QApplication::quit();
}

void AutoscoperMainWindow::on_actionSave_Test_Sequence_triggered(bool checked){
	fprintf(stderr,"Saving testSequence\n");
	for (int i = 0 ; i < tracker->trial()->num_frames;i ++){
		timeline_widget->setFrame(i);
		for(int j = 0 ; j < cameraViews.size(); j++){
			QFileInfo fi (cameraViews[j]->getName());
			if(i == 0){
				QDir dir (fi.absolutePath() + OS_SEP + fi.completeBaseName());
				if (!dir.exists()){
					dir.mkdir(".");
				}
			}
			QString filename = fi.absolutePath() + OS_SEP + fi.completeBaseName() + OS_SEP +  fi.completeBaseName() + QString().sprintf("%05d", i) + ".pgm";
			cameraViews[j]->saveFrame(filename);
		}
		QApplication::processEvents();
	}
}

//Edit menu

void AutoscoperMainWindow::on_actionUndo_triggered(bool checked){
	undo_state();
}

void AutoscoperMainWindow::on_actionRedo_triggered(bool checked){
	redo_state();
}

void AutoscoperMainWindow::on_actionCut_triggered(bool checked){
	push_state();

    if (!timeline_widget->getSelectedNodes()->empty()) {
        timeline_widget->getCopiedNodes()->clear();
        for (unsigned i = 0; i < timeline_widget->getSelectedNodes()->size(); i++) {
            if ((*timeline_widget->getSelectedNodes())[i].second == NODE) {
				timeline_widget->getCopiedNodes()->push_back( (*timeline_widget->getSelectedNodes())[i].first);
                if (!timeline_widget->getPosition_graph()->frame_locks.at((int)(*timeline_widget->getSelectedNodes())[i].first.first->time((*timeline_widget->getSelectedNodes())[i].first.second))) {
                    (*timeline_widget->getSelectedNodes())[i].first.first->erase((*timeline_widget->getSelectedNodes())[i].first.second);
                }
            }
        }
        timeline_widget->getSelectedNodes()->clear();
    }

    update_xyzypr_and_coord_frame();
	update_graph_min_max(timeline_widget->getPosition_graph());

    redrawGL();
}

void AutoscoperMainWindow::on_actionCopy_triggered(bool checked){
	if (!timeline_widget->getSelectedNodes()->empty()) {
		timeline_widget->getCopiedNodes()->clear();
        for (unsigned i = 0; i < timeline_widget->getSelectedNodes()->size(); i++) {
            if ((*timeline_widget->getSelectedNodes())[i].second == NODE) {
                timeline_widget->getCopiedNodes()->push_back((*timeline_widget->getSelectedNodes())[i].first);
            }
        }
    }

	redrawGL();
}

void AutoscoperMainWindow::on_actionPaste_triggered(bool checked){
	push_state();

    if (!timeline_widget->getCopiedNodes()->empty()) {
        float frame_offset = timeline_widget->getCopiedNodes()->front().first->time(timeline_widget->getCopiedNodes()->front().second);
        for (unsigned i = 0; i < timeline_widget->getCopiedNodes()->size(); i++) {
			float frame = tracker->trial()->frame+(*timeline_widget->getCopiedNodes())[i].first->time((*timeline_widget->getCopiedNodes())[i].second)-frame_offset;
			if (!timeline_widget->getPosition_graph()->frame_locks.at((int)frame)) {
				if((*timeline_widget->getCopiedNodes())[i].first->type == KeyCurve::X_CURVE){
				   getTracker()->trial()->getXCurve(-1)->insert(frame,(*timeline_widget->getCopiedNodes())[i].first->value((*timeline_widget->getCopiedNodes())[i].second));
				}else if((*timeline_widget->getCopiedNodes())[i].first->type == KeyCurve::Y_CURVE){
				   getTracker()->trial()->getYCurve(-1)->insert(frame,(*timeline_widget->getCopiedNodes())[i].first->value((*timeline_widget->getCopiedNodes())[i].second));
				}else if((*timeline_widget->getCopiedNodes())[i].first->type == KeyCurve::Z_CURVE){
				   getTracker()->trial()->getZCurve(-1)->insert(frame,(*timeline_widget->getCopiedNodes())[i].first->value((*timeline_widget->getCopiedNodes())[i].second));
				}else if((*timeline_widget->getCopiedNodes())[i].first->type == KeyCurve::YAW_CURVE){
				   getTracker()->trial()->getYawCurve(-1)->insert(frame,(*timeline_widget->getCopiedNodes())[i].first->value((*timeline_widget->getCopiedNodes())[i].second));
				}else if((*timeline_widget->getCopiedNodes())[i].first->type == KeyCurve::PITCH_CURVE){
				   getTracker()->trial()->getPitchCurve(-1)->insert(frame,(*timeline_widget->getCopiedNodes())[i].first->value((*timeline_widget->getCopiedNodes())[i].second));
				}else if((*timeline_widget->getCopiedNodes())[i].first->type == KeyCurve::ROLL_CURVE){
				   getTracker()->trial()->getRollCurve(-1)->insert(frame,(*timeline_widget->getCopiedNodes())[i].first->value((*timeline_widget->getCopiedNodes())[i].second));
				}
				//(*timeline_widget->getCopiedNodes())[i].first->insert(frame,(*timeline_widget->getCopiedNodes())[i].first->value((*timeline_widget->getCopiedNodes())[i].second));
            }
        }
        timeline_widget->getSelectedNodes()->clear();
    }

    update_xyzypr_and_coord_frame();
    update_graph_min_max(timeline_widget->getPosition_graph());

	redrawGL();
}

void AutoscoperMainWindow::on_actionDelete_triggered(bool checked){
	if (!timeline_widget->getSelectedNodes()->empty()) {
        push_state();

        for (unsigned i = 0; i < timeline_widget->getSelectedNodes()->size(); i++) {
            if ((*timeline_widget->getSelectedNodes())[i].second == NODE) {
				if (!timeline_widget->getPosition_graph()->frame_locks.at((int)(*timeline_widget->getSelectedNodes())[i].first.first->time((*timeline_widget->getSelectedNodes())[i].first.second))) {
                    (*timeline_widget->getSelectedNodes())[i].first.first->erase((*timeline_widget->getSelectedNodes())[i].first.second);
                }
            }
        }
        timeline_widget->getSelectedNodes()->clear();

        update_xyzypr_and_coord_frame();
        update_graph_min_max(timeline_widget->getPosition_graph());

		redrawGL();
    }
}

//Tracking menu
void AutoscoperMainWindow::on_actionImport_triggered(bool checked){
	QString filename = get_filename(false,"*.tra");
    if (filename.compare("") != 0) {
        load_tracking_results(filename);
    }
}

void AutoscoperMainWindow::on_actionExport_triggered(bool checked){
	QString filename = get_filename(true, "*.tra");
    if (filename.compare("") != 0) {
        save_tracking_results(filename);
    }
}

void AutoscoperMainWindow::on_actionInsert_Key_triggered(bool checked){
	push_state();
	timeline_widget->getSelectedNodes()->clear();

    double xyzypr[6];
    //(CoordFrame::from_matrix(trans(getManipulator()->transform()))* *getVolume_matrix()).to_xyzypr(xyzypr);
    (CoordFrame::from_matrix(trans(getManipulator()->transform()))* *tracker->trial()->getVolumeMatrix(-1)).to_xyzypr(xyzypr);
    getTracker()->trial()->getXCurve(-1)->insert(getTracker()->trial()->frame,xyzypr[0]);
    getTracker()->trial()->getYCurve(-1)->insert(getTracker()->trial()->frame,xyzypr[1]);
    getTracker()->trial()->getZCurve(-1)->insert(getTracker()->trial()->frame,xyzypr[2]);
    getTracker()->trial()->getYawCurve(-1)->insert(getTracker()->trial()->frame,xyzypr[3]);
    getTracker()->trial()->getPitchCurve(-1)->insert(getTracker()->trial()->frame,xyzypr[4]);
    getTracker()->trial()->getRollCurve(-1)->insert(getTracker()->trial()->frame,xyzypr[5]);

	timeline_widget->update_graph_min_max();

	redrawGL();
}

void AutoscoperMainWindow::on_actionLock_triggered(bool checked){
	push_state();

    for (unsigned i = 0; i < timeline_widget->getSelectedNodes()->size(); i++) {

        int time = (int)(*timeline_widget->getSelectedNodes())[i].first.first->time(
                   (*timeline_widget->getSelectedNodes())[i].first.second);

        // Force the addition of keys for all curves in order to truely freeze
        // the frame

        tracker->trial()->getXCurve(-1)->insert(time);
        tracker->trial()->getYCurve(-1)->insert(time);
        tracker->trial()->getZCurve(-1)->insert(time);
        tracker->trial()->getYawCurve(-1)->insert(time);
        tracker->trial()->getPitchCurve(-1)->insert(time);
        tracker->trial()->getRollCurve(-1)->insert(time);

        timeline_widget->getPosition_graph()->frame_locks.at(time) = true;
    }

    timeline_widget->getSelectedNodes()->clear();

    frame_changed();
    redrawGL();
}

void AutoscoperMainWindow::on_actionUnlock_triggered(bool checked){
	for (unsigned i = 0; i < timeline_widget->getSelectedNodes()->size(); i++) {
		timeline_widget->getPosition_graph()->frame_locks.at((int)(*timeline_widget->getSelectedNodes())[i].first.first->time(
                                   (*timeline_widget->getSelectedNodes())[i].first.second)) = false;
    }

    frame_changed();
    redrawGL();
}

void AutoscoperMainWindow::on_actionBreak_Tangents_triggered(bool checked){
	push_state();

    for (unsigned i = 0; i < timeline_widget->getSelectedNodes()->size(); i++) {
        KeyCurve& curve = *(*timeline_widget->getSelectedNodes())[i].first.first;
        KeyCurve::iterator it = (*timeline_widget->getSelectedNodes())[i].first.second;
        if (!timeline_widget->getPosition_graph()->frame_locks.at((int)curve.time(it))) {
            curve.set_bind_tangents(it,false);
        }
    }

}

void AutoscoperMainWindow::on_actionSmooth_Tangents_triggered(bool checked){
	push_state();

    for (unsigned i = 0; i < timeline_widget->getSelectedNodes()->size(); i++) {
        KeyCurve& curve = *(*timeline_widget->getSelectedNodes())[i].first.first;
        KeyCurve::iterator it = (*timeline_widget->getSelectedNodes())[i].first.second;

        if (!timeline_widget->getPosition_graph()->frame_locks.at((int)curve.time(it))) {
            curve.set_bind_tangents(it,true);
            curve.set_in_tangent_type(it,KeyCurve::SMOOTH);
            curve.set_out_tangent_type(it,KeyCurve::SMOOTH);
        }
    }

    update_xyzypr_and_coord_frame();
    update_graph_min_max(timeline_widget->getPosition_graph());

    redrawGL();
}

//View

void AutoscoperMainWindow::on_actionLayoutCameraViews_triggered(bool triggered){
	
	bool ok;
    int rows = QInputDialog::getInteger(this, tr("Layput Camera Views"),
                                          tr("Number of Rows"),1,1,10,1, &ok);
    if (ok)relayoutCameras(rows);
}

//Toolbar

void AutoscoperMainWindow::on_toolButtonOpenTrial_clicked(){
	save_trial_prompt();
    save_tracking_prompt();
	
	openTrial();
}

void AutoscoperMainWindow::on_toolButtonSaveTracking_clicked(){
	QString filename = get_filename(true, "*.tra");
    if (filename.compare("") != 0) {
        save_tracking_results(filename);
    }
}

void AutoscoperMainWindow::on_toolButtonLoadTracking_clicked(){
	QString filename = get_filename(false, "*.tra");
    if (filename.compare("") != 0) {
        load_tracking_results(filename);
    }
}

void AutoscoperMainWindow::on_toolButtonTranslate_clicked(){
	ui->toolButtonTranslate->setChecked(true);
	ui->toolButtonRotate->setChecked(false);
	getManipulator()->set_mode(Manip3D::TRANSLATION);
	redrawGL();
}

void AutoscoperMainWindow::on_toolButtonRotate_clicked(){
	ui->toolButtonRotate->setChecked(true);
	ui->toolButtonTranslate->setChecked(false);
	getManipulator()->set_mode(Manip3D::ROTATION);
	redrawGL();
}

void AutoscoperMainWindow::on_toolButtonMovePivot_clicked(){
	getManipulator()->set_movePivot(ui->toolButtonMovePivot->isChecked());
	redrawGL();
}

void AutoscoperMainWindow::on_toolButtonTrack_clicked(){
	if(tracking_dialog == NULL)tracking_dialog = new TrackingOptionsDialog(this);

	tracking_dialog->setRange(timeline_widget->getPosition_graph()->min_frame,timeline_widget->getPosition_graph()->max_frame, 
		tracker->trial()->num_frames-1);
	
	tracking_dialog->show();
}

void AutoscoperMainWindow::on_toolButtonRetrack_clicked(){
	if(tracking_dialog == NULL)tracking_dialog = new TrackingOptionsDialog(this);

	tracking_dialog->setRange(timeline_widget->getPosition_graph()->min_frame,timeline_widget->getPosition_graph()->max_frame, 
		tracker->trial()->num_frames-1);
	
	tracking_dialog->retrack();
}

void AutoscoperMainWindow::key_w_pressed(){
	ui->toolButtonTranslate->click();
}
void AutoscoperMainWindow::key_e_pressed(){
	ui->toolButtonRotate->click();
}	
void AutoscoperMainWindow::key_d_pressed(){
	ui->toolButtonMovePivot->click();
}	
void AutoscoperMainWindow::key_h_pressed(){
	filters_widget->toggle_drrs();
}
void AutoscoperMainWindow::key_t_pressed(){
	ui->toolButtonTrack->click();
}	
void AutoscoperMainWindow::key_r_pressed(){
	ui->toolButtonRetrack->click();
}	
void AutoscoperMainWindow::key_plus_pressed(){
	getManipulator(-1)->set_pivotSize(getManipulator(-1)->get_pivotSize() * 1.1f);
	redrawGL();
}	
void AutoscoperMainWindow::key_equal_pressed(){
	getManipulator(-1)->set_pivotSize(getManipulator(-1)->get_pivotSize() * 1.1f);
	redrawGL();
}	

void AutoscoperMainWindow::key_minus_pressed(){
	getManipulator(-1)->set_pivotSize(getManipulator(-1)->get_pivotSize() * 0.9f);
	redrawGL();
}

//Shortcuts
void AutoscoperMainWindow::setupShortcuts(){
	new QShortcut(QKeySequence(Qt::Key_W), this, SLOT(key_w_pressed()));
	new QShortcut(QKeySequence(Qt::Key_E), this, SLOT(key_e_pressed()));
	new QShortcut(QKeySequence(Qt::Key_D), this, SLOT(key_d_pressed()));
	new QShortcut(QKeySequence(Qt::Key_H), this, SLOT(key_h_pressed()));
	new QShortcut(QKeySequence(Qt::Key_T), this, SLOT(key_t_pressed()));
	new QShortcut(QKeySequence(Qt::Key_R), this, SLOT(key_r_pressed()));
	new QShortcut(QKeySequence(Qt::Key_Plus), this, SLOT(key_plus_pressed()));
	new QShortcut(QKeySequence(Qt::Key_Equal), this, SLOT(key_equal_pressed()));
	new QShortcut(QKeySequence(Qt::Key_Minus), this, SLOT(key_minus_pressed()));

	ui->actionCopy->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_C));
	ui->actionPaste->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_V));
	ui->actionCut->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_X));
	ui->actionDelete->setShortcut(QKeySequence(Qt::Key_Delete));
	ui->actionUndo->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_Z));
	ui->actionRedo->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_Y));

	ui->actionNew->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_N));
	ui->actionOpen->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_O));
	ui->actionSave->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_S));
	ui->actionSave_as->setShortcut(QKeySequence(Qt::SHIFT + Qt::CTRL + Qt::Key_S));
	ui->actionQuit->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_Q));
	ui->actionInsert_Key->setShortcut(QKeySequence(Qt::Key_S));
}