#ifdef _MSC_VER
#define _CRT_SECURE_NO_WARNINGS
#endif

#define GL_GLEXT_PROTOTYPES 1
#include <GL/glew.h>

#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#else
#ifdef _WIN32
  #include <windows.h>
#endif
#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#include "ui/GLView.h"
#include "ui/AutoscoperMainWindow.h"
#include "ui/CameraViewWidget.h"
#include "ui/TimelineDockWidget.h"

#include "Tracker.hpp"
#include "View.hpp"
#include "Trial.hpp"

#include "Manip3D.hpp"


#ifdef WITH_CUDA
#include <gpu/cuda/RadRenderer.hpp>
#include <gpu/cuda/RayCaster.hpp>
#else
#include <gpu/opencl/RadRenderer.hpp>
#include <gpu/opencl/RayCaster.hpp>
#endif

#include <QMouseEvent>
#include <QWheelEvent>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

GLView::GLView(QWidget *parent)
    : GLWidget(parent)
{
	m_view = NULL;
	viewdata.m_isStaticView = false;
	double xyzypr[6] = {250.0f, 250.0f, 250.0f, 0.0f, 45.0f, -35.0f};
	defaultViewMatrix = CoordFrame::from_xyzypr(xyzypr);
}

void GLView::setStaticView(bool staticView){
	viewdata.m_isStaticView = staticView;
	if(viewdata.m_isStaticView){
		const double xyzypr[6] = {250.0f, 250.0f, 250.0f, 0.0f, 45.0f, -35.0f};
		defaultViewMatrix = CoordFrame::from_xyzypr(xyzypr);
	}
}

void GLView::setView(View * view){
	m_view = view;

	viewdata.ratio = 1.0f;
    viewdata.fovy = 53.13f;
    viewdata.near_clip = 1.0f;
    viewdata.far_clip = 10000.0f;
}

// Selects the axis of translation or rotation of the manipulator that is under
// the mouse located at pixel coordinates x,y.
void GLView::select_manip_in_view(double x, double y, int button)
{
	CameraViewWidget * cameraViewWidget = dynamic_cast <CameraViewWidget *> ( this->parent());

    // Setup the view from this perspective so that we can simply call set_view
    // on the manipulator
    glViewport(viewdata.viewport_x,
               viewdata.viewport_y,
               viewdata.viewport_width,
               viewdata.viewport_height);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(viewdata.fovy,viewdata.ratio,viewdata.near_clip,viewdata.far_clip);

	CoordFrame viewMatrix;
	if (viewdata.m_isStaticView) {
        viewMatrix = defaultViewMatrix;
	}else{
		viewMatrix = cameraViewWidget->getMainWindow()->getTracker()->trial()->cameras.at(cameraViewWidget->getID()).coord_frame();
	}
    double m[16];
    viewMatrix.inverse().to_matrix(m);

    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixd(m);

    cameraViewWidget->getMainWindow()->getManipulator()->set_view();
    cameraViewWidget->getMainWindow()->getManipulator()->set_size(viewdata.scale*cameraViewWidget->getMainWindow()->getManipulator()->get_pivotSize());
    cameraViewWidget->getMainWindow()->getManipulator()->on_mouse_press(x,viewdata.window_height-y);
}

 void GLView::wheelEvent(QWheelEvent *e)
 {
	 CameraViewWidget * cameraViewWidget = dynamic_cast <CameraViewWidget *> ( this->parent());

	 if ( Qt::ControlModifier & e->modifiers() ) {
		 if (e->delta() > 0) {
            viewdata.zoom *= 1.1f;
        }
        else if (e->delta() < 0) {
            viewdata.zoom /= 1.1f;
        }

        update_viewport(&viewdata);
        cameraViewWidget->getMainWindow()->redrawGL();
    }
 }

void GLView::mousePressEvent(QMouseEvent *e){
	CameraViewWidget * cameraViewWidget = dynamic_cast <CameraViewWidget *> ( this->parent());

	press_x = e->x();
    press_y = e->y();
	prevx = e->x();
	prevy = e->y();

    select_manip_in_view(e->x(),e->y(),e->button());

	cameraViewWidget->getMainWindow()->redrawGL();
}

// Moves the manipulator and volume based on the view, the selected axis, and
// the direction of the motion.
void GLView::move_manip_in_view(double x, double y, bool out_of_plane)
{
	CameraViewWidget * cameraViewWidget = dynamic_cast <CameraViewWidget *> ( this->parent());
	AutoscoperMainWindow * mainwindow = cameraViewWidget->getMainWindow();

	if (mainwindow->getPosition_graph()->frame_locks.at(mainwindow->getTracker()->trial()->frame)) {
        return;
    }

    CoordFrame frame;
	if (mainwindow->getManipulator()->get_movePivot()) {
        //frame = (CoordFrame::from_matrix(trans(mainwindow->getManipulator()->transform()))* *mainwindow->getVolume_matrix());
		frame = (CoordFrame::from_matrix(trans(mainwindow->getManipulator()->transform()))* *mainwindow->getTracker()->trial()->getVolumeMatrix(-1));
	}

    if (!out_of_plane) {
		mainwindow->getManipulator()->set_size(viewdata.scale*mainwindow->getManipulator()->get_pivotSize());
        mainwindow->getManipulator()->on_mouse_move(x,viewdata.window_height-y);
    }
    else if (mainwindow->getManipulator()->selection() == Manip3D::VIEW_PLANE) {
		CoordFrame mmat = CoordFrame::from_matrix(trans(mainwindow->getManipulator()->transform()));
		
		CoordFrame viewMatrix;
		if (viewdata.m_isStaticView) {
			viewMatrix = defaultViewMatrix;
		}else{
			viewMatrix = cameraViewWidget->getMainWindow()->getTracker()->trial()->cameras.at(cameraViewWidget->getID()).coord_frame();
		}


        double zdir[3] = { mmat.translation()[0]-viewMatrix.translation()[0],
                           mmat.translation()[1]-viewMatrix.translation()[1],
                           mmat.translation()[2]-viewMatrix.translation()[2]};
        double mag = sqrt(zdir[0]*zdir[0]+zdir[1]*zdir[1]+zdir[2]*zdir[2]);
        zdir[0] /= mag;
        zdir[1] /= mag;
        zdir[2] /= mag;

        double ztrans[3] = { (x-y)/2.0*zdir[0],(x-y)/2.0*zdir[1],(x-y)/2.0*zdir[2] };

        mmat.translate(ztrans);

		double m[16];
		mmat.to_matrix_row_order(m);
		mainwindow->getManipulator()->set_transform(Mat4d(m));

        mainwindow->getManipulator()->set_selection(Manip3D::VIEW_PLANE);
    }
	
	if (mainwindow->getManipulator()->get_movePivot()) {
        CoordFrame new_manip_matrix = CoordFrame::from_matrix(trans(mainwindow->getManipulator()->transform()));
		//mainwindow->setVolume_matrix(new_manip_matrix.inverse()*frame);
		*(mainwindow->getTracker()->trial()->getVolumeMatrix(-1)) = new_manip_matrix.inverse()*frame;
    }
}

void GLView::mouseMoveEvent(QMouseEvent *e){
	double dx = e->x() - prevx;
    double dy = e->y() - prevy;

	double x = e->x();
	double y = e->y();
	if ( Qt::ControlModifier & e->modifiers() ) {
		if (viewdata.m_isStaticView) {
            if (e->buttons() &  Qt::LeftButton) {
                CoordFrame rotationMatrix;
                rotationMatrix.rotate(defaultViewMatrix.rotation()+3,
                                             -dx/2.0);
                rotationMatrix.rotate(defaultViewMatrix.rotation()+0,
                                             -dy/2.0);

                defaultViewMatrix = rotationMatrix*defaultViewMatrix;
            }
            else if (e->buttons() &  Qt::MiddleButton) {
                double xtrans[3] = {-dx*defaultViewMatrix.rotation()[0],
                                    -dx*defaultViewMatrix.rotation()[1],
                                    -dx*defaultViewMatrix.rotation()[2]};
                double ytrans[3] = {dy*defaultViewMatrix.rotation()[3],
                                    dy*defaultViewMatrix.rotation()[4],
                                    dy*defaultViewMatrix.rotation()[5]};

                defaultViewMatrix.translate(xtrans);
                defaultViewMatrix.translate(ytrans);
            }
			else if (e->buttons() &  Qt::RightButton) {
                double ztrans[3] =
                    { (dx-dy)/2.0*defaultViewMatrix.rotation()[6],
                      (dx-dy)/2.0*defaultViewMatrix.rotation()[7],
                      (dx-dy)/2.0*defaultViewMatrix.rotation()[8] };

                defaultViewMatrix.translate(ztrans);
            }
        }
        else {
            if (e->buttons() &  Qt::LeftButton) {
				viewdata.zoom_x -= dx/200/viewdata.zoom;
				viewdata.zoom_y += dy/200/viewdata.zoom;

				update_viewport(&viewdata);
			}
		}
        update_scale_in_view(&viewdata);
    }
    else {
        if (Qt::ShiftModifier & e->modifiers()) {
            if (e->buttons() & Qt::LeftButton) {
                // Only display in one direction
				if (abs(e->x()-press_x) > abs(e->y()-press_y)) {
                    y = press_y;
                }
                else {
                   x = press_x;
                }
                move_manip_in_view(x,y);
            }
        }
        else {
            if (e->buttons() &  Qt::LeftButton) {
                move_manip_in_view(x,y);
            }
			else if (e->buttons() & Qt::RightButton) {
                move_manip_in_view(dx,dy,true);
            }
        }
    }
	CameraViewWidget * cameraViewWidget = dynamic_cast <CameraViewWidget *> ( this->parent());
	AutoscoperMainWindow * mainwindow = cameraViewWidget->getMainWindow();

    mainwindow->update_xyzypr();

    prevx = x;
    prevy = y;
}

void GLView::mouseReleaseEvent(QMouseEvent *e){
	CameraViewWidget * cameraViewWidget = dynamic_cast <CameraViewWidget *> ( this->parent());
   
	cameraViewWidget->getMainWindow()->getManipulator()->on_mouse_release(e->x(),e->y());

    cameraViewWidget->getMainWindow()->update_graph_min_max(cameraViewWidget->getMainWindow()->getTracker()->trial()->frame);

	cameraViewWidget->getMainWindow()->redrawGL();
}

void GLView::paintGL()
{
	CameraViewWidget * cameraViewWidget = dynamic_cast <CameraViewWidget *> ( this->parent());
	AutoscoperMainWindow * mainwindow = cameraViewWidget->getMainWindow();

	if(mainwindow && cameraViewWidget){
		update_scale_in_view(&viewdata);
		update_viewport(&viewdata);
		
		glViewport(viewdata.viewport_x,
				   viewdata.viewport_y,
				   viewdata.viewport_width,
				   viewdata.viewport_height);

		double m[16];

		if(viewdata.m_isStaticView){
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

			defaultViewMatrix.inverse().to_matrix(m);

			glMatrixMode(GL_PROJECTION);
			glLoadIdentity();
			gluPerspective(viewdata.fovy,viewdata.ratio,viewdata.near_clip,viewdata.far_clip);
			glMatrixMode(GL_MODELVIEW);
			glLoadMatrixd(m);

			// Draw background
			float top_color[3] = {0.20f,0.35f,0.50f};
			float bot_color[3] = {0.10f,0.17f,0.25f};

			draw_gradient(top_color,bot_color);	
			// Draw image planes
			for (unsigned int i = 0; i < mainwindow->getTracker()->trial()->cameras.size(); ++i) {
				draw_textured_quad(mainwindow->getTracker()->trial()->cameras.at(i).image_plane(),
					(*mainwindow->getTextures())[i]);
			}

			// Draw cameras
			enable_headlight();
			glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
			for (unsigned int i = 0; i < mainwindow->getTracker()->trial()->cameras.size(); ++i) {

				glPushMatrix();

				double m1[16];
				mainwindow->getTracker()->trial()->cameras.at(i).coord_frame().to_matrix(m1);
				glMultMatrixd(m1);

				float scale = 0.05*sqrt(m1[12]*m1[12]+m1[13]*m1[13]+m1[14]*m1[14]);
				glScalef(scale,scale,scale);

				glColor3f(0.5f, 0.5f, 0.5f);
				draw_camera();

				glPopMatrix();
			}

			draw_manip_from_view(&viewdata);
			glDisable(GL_LIGHTING);

			// Draw grid
			bool drawGrid = true;
			if (drawGrid == true) {
				draw_xz_grid(24, 24, 10.0f);
			}

			if (!mainwindow->getTracker()->views().empty()) {
				float width = 2.0f/viewdata.zoom, height = 2.0f/viewdata.zoom;
				float x = viewdata.zoom_x-width/2.0f, y = viewdata.zoom_y-height/2.0f;

				for(int idx_volume = 0 ; idx_volume < mainwindow->getTracker()->trial()->num_volumes ; idx_volume++){
					CoordFrame modelview = defaultViewMatrix.inverse()*CoordFrame::from_matrix(trans(mainwindow->getManipulator(idx_volume)->transform()))* *mainwindow->getTracker()->trial()->getVolumeMatrix(idx_volume);

					double imv[16];
					modelview.inverse().to_matrix_row_order(imv);
					mainwindow->getTracker()->view(0)->drrRenderer(idx_volume)->setInvModelView(imv);

					mainwindow->getTracker()->view(0)->drrRenderer(idx_volume)->setViewport(
					viewdata.ratio*x, y, viewdata.ratio*width, height);
				}

				mainwindow->getTracker()->view(0)->renderDrr(viewdata.pbo,viewdata.window_width,viewdata.window_height);

				glViewport(0, 0, viewdata.window_width, viewdata.window_height);

				glMatrixMode(GL_PROJECTION);
				glLoadIdentity();
				glOrtho(0.0, 1.0, 0.0, 1.0, 0.0, 1.0);

				glMatrixMode(GL_MODELVIEW);
				glLoadIdentity();

				glDisable(GL_DEPTH_TEST);
				glEnable(GL_BLEND);
				glBlendFunc(GL_ONE,GL_ONE);

				CALL_GL(glRasterPos2i(0, 0));

	#ifdef WITH_CUDA
				CALL_GL(glBindBufferARB(GL_PIXEL_UNPACK_BUFFER_ARB, viewdata.pbo));
				CALL_GL(glDrawPixels(viewdata.window_width,
							 viewdata.window_height,
							 GL_RGB, GL_FLOAT, 0));
				CALL_GL(glBindBufferARB(GL_PIXEL_UNPACK_BUFFER_ARB, 0));    
	#else
				CALL_GL(glBindBuffer(GL_PIXEL_UNPACK_BUFFER, viewdata.pbo));
				CALL_GL(glDrawPixels(viewdata.window_width,
							 viewdata.window_height,
							 GL_RGB, GL_FLOAT, 0));
				CALL_GL(glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0));
	#endif
				CALL_GL(glDisable(GL_BLEND));
				CALL_GL(glEnable(GL_DEPTH_TEST));
			}
			return;
		}
		else{
			float temp = 2.0f*sqrt(5.0)*sin(M_PI*viewdata.fovy/360.0);
			float width = temp/viewdata.zoom, height = temp/viewdata.zoom;
			float x = viewdata.zoom_x-width/2.0f, y = viewdata.zoom_y-height/2.0f;

			for(int idx_volume = 0 ; idx_volume < mainwindow->getTracker()->trial()->num_volumes ; idx_volume++){
				CoordFrame modelview  = mainwindow->getTracker()->trial()->cameras.at(cameraViewWidget->getID()).coord_frame().inverse()
									* CoordFrame::from_matrix(trans(mainwindow->getManipulator(idx_volume)->transform()))
									* (*mainwindow->getTracker()->trial()->getVolumeMatrix(idx_volume));
				double imv[16];
				modelview.inverse().to_matrix_row_order(imv);
				int idx = mainwindow->getTracker()->trial()->current_volume;
				m_view->drrRenderer(idx_volume)->setInvModelView(imv);

				m_view->drrRenderer(idx_volume)->setViewport(
				viewdata.ratio*x, y, viewdata.ratio*width, height);
			}
			
			m_view->radRenderer()->set_viewport(
				viewdata.ratio*x, y, viewdata.ratio*width, height);

			m_view->render(viewdata.pbo,viewdata.window_width, viewdata.window_height);

			glViewport(0, 0,viewdata.window_width, viewdata.window_height);

			glMatrixMode(GL_PROJECTION);
			glLoadIdentity();
			glOrtho(0.0, 1.0, 0.0, 1.0, 0.0, 1.0);

			glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();
		
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			glDisable(GL_DEPTH_TEST);
			glRasterPos2i(0, 0);

			#ifdef WITH_CUDA
			CALL_GL(glBindBufferARB(GL_PIXEL_UNPACK_BUFFER_ARB, view->pbo));
			CALL_GL(glDrawPixels(view->window_width,
							view->window_height,
							GL_RGB, GL_FLOAT, 0));
			CALL_GL(glBindBufferARB(GL_PIXEL_UNPACK_BUFFER_ARB, 0));
			#else
			CALL_GL(glBindBuffer(GL_PIXEL_UNPACK_BUFFER, viewdata.pbo));
			CALL_GL(glDrawPixels(viewdata.window_width,
							viewdata.window_height,
							GL_RGB, GL_FLOAT, 0));
			CALL_GL(glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0));
			#endif
		
			glEnable(GL_DEPTH_TEST);

			glViewport(viewdata.viewport_x,
						viewdata.viewport_y,
						viewdata.viewport_width,
						viewdata.viewport_height);

			mainwindow->getTracker()->trial()->cameras.at(cameraViewWidget->getID()).
				coord_frame().inverse().to_matrix(m);

			glMatrixMode(GL_PROJECTION);
			glLoadIdentity();
			gluPerspective(viewdata.fovy,viewdata.ratio,viewdata.near_clip,viewdata.far_clip);

			for(int idx_volume = 0 ; idx_volume < mainwindow->getTracker()->trial()->num_volumes ; idx_volume++){
				CoordFrame modelview  = mainwindow->getTracker()->trial()->cameras.at(cameraViewWidget->getID()).coord_frame().inverse()
									* CoordFrame::from_matrix(trans(mainwindow->getManipulator(idx_volume)->transform()))
									* (*mainwindow->getTracker()->trial()->getVolumeMatrix(idx_volume));
				if (mainwindow->getTracker()->show2DBoundingBox){
					drawViewport(modelview, idx_volume);
				}
				if (mainwindow->getTracker()->show3DBoundingBox){
					drawBB(modelview, idx_volume);
				}
				if (mainwindow->getTracker()->show2DProjectionBox){
					draw2DProjectionBB(modelview, idx_volume);
				}
				if (mainwindow->getTracker()->showModelCoordinateAxes){
					drawLocalCoordAxes(modelview, idx_volume);
				}
			}

			glMatrixMode(GL_MODELVIEW);
			glLoadMatrixd(m);

			enable_headlight();
			draw_manip_from_view(&viewdata);

			glDisable(GL_LIGHTING);
		}
	}
}

void GLView::saveView(std::string filename){
	CameraViewWidget * cameraViewWidget = dynamic_cast <CameraViewWidget *> ( this->parent());
	AutoscoperMainWindow * mainwindow = cameraViewWidget->getMainWindow();

	// Calculate the minimum and maximum values of the bounding box
	// corners after they have been projected onto the view plane
	double min_max[4] = {1.0,1.0,-1.0,-1.0};

	for (int j = 0; j < 4; j++) {
		// Calculate the location of the corner in camera space
		double corner[3];
			m_view->camera()->coord_frame().inverse().point_to_world_space( &m_view->camera()->image_plane()[3*j],corner);

		// Calculate its projection onto the film plane, where z = -2
		double film_plane[3];
		film_plane[0] = -2*corner[0]/corner[2];
		film_plane[1] = -2*corner[1]/corner[2];

		// Update the min and max values
		if (min_max[0] > film_plane[0]) {
			min_max[0] = film_plane[0];
		}
		if (min_max[1] > film_plane[1]) {
			min_max[1] = film_plane[1];
		}
		if (min_max[2] < film_plane[0]) {
			min_max[2] = film_plane[0];
		}
		if (min_max[3] < film_plane[1]) {
			min_max[3] = film_plane[1];
		}
	}

	double viewport[4];
	viewport[0] = min_max[0];
	viewport[1] = min_max[1];
	viewport[2] = min_max[2]-min_max[0];
	viewport[3] = min_max[3]-min_max[1];

	for(int idx_volume = 0 ; idx_volume < mainwindow->getTracker()->trial()->num_volumes ; idx_volume++){
		CoordFrame modelview  = mainwindow->getTracker()->trial()->cameras.at(cameraViewWidget->getID()).coord_frame().inverse()
							* CoordFrame::from_matrix(trans(mainwindow->getManipulator(idx_volume)->transform()))
							* (*mainwindow->getTracker()->trial()->getVolumeMatrix(idx_volume));
		double imv[16];
		modelview.inverse().to_matrix_row_order(imv);
		int idx = mainwindow->getTracker()->trial()->current_volume;
		m_view->drrRenderer(idx_volume)->setInvModelView(imv);

		m_view->drrRenderer(idx_volume)->setViewport(viewport[0],viewport[1],
                                        viewport[2],viewport[3]);
	}

	m_view->saveImage(filename,m_view->camera()->image_width,m_view->camera()->image_height);
}

void GLView::update_scale_in_view(ViewData* view)
{
    // Determine the distance from the center of the pivot point to the
    // center of the view.
	CameraViewWidget * cameraViewWidget = dynamic_cast <CameraViewWidget *> ( this->parent());
	AutoscoperMainWindow * mainwindow = cameraViewWidget->getMainWindow();
	
	CoordFrame mat = CoordFrame::from_matrix(trans(cameraViewWidget->getMainWindow()->getManipulator()->transform()));
    
	double dist_vec[3];
	if (view->m_isStaticView) {
        dist_vec[0] = mat.translation()[0]-
                      defaultViewMatrix.translation()[0];
        dist_vec[2] = mat.translation()[1]-
                      defaultViewMatrix.translation()[1];
        dist_vec[1] = mat.translation()[2]-
                      defaultViewMatrix.translation()[2];
    }
    else {
		dist_vec[0] = mat.translation()[0]-
			mainwindow->getTracker()->trial()->cameras.at(cameraViewWidget->getID()).coord_frame().translation()[0];
		dist_vec[1] = mat.translation()[1]-
			mainwindow->getTracker()->trial()->cameras.at(cameraViewWidget->getID()).coord_frame().translation()[1];
		dist_vec[2] = mat.translation()[2]-
			mainwindow->getTracker()->trial()->cameras.at(cameraViewWidget->getID()).coord_frame().translation()[2];
	}
    double dist = sqrt(dist_vec[0]*dist_vec[0]+
                       dist_vec[1]*dist_vec[1]+
                       dist_vec[2]*dist_vec[2]);

    // Adjust the size of the pivot based on the distance.
    view->scale = 2.0*dist*tan(view->fovy*M_PI/360.0)*view->near_clip/view->zoom;
}

void GLView::draw_manip_from_view(const ViewData* view)
{
	CameraViewWidget * cameraViewWidget = dynamic_cast <CameraViewWidget *> ( this->parent());
   
	if ( cameraViewWidget->getMainWindow()->getManipulator()->get_movePivot()) {
        glEnable(GL_LINE_STIPPLE);
        glLineStipple(2,0x3333);
    }
	
	glLineWidth(1.0);
	cameraViewWidget->getMainWindow()->getManipulator()->set_size(view->scale*cameraViewWidget->getMainWindow()->getManipulator()->get_pivotSize());
    cameraViewWidget->getMainWindow()->getManipulator()->draw();

   if ( cameraViewWidget->getMainWindow()->getManipulator()->get_movePivot()) {
        glLineStipple(1,0);
        glDisable(GL_LINE_STIPPLE);
    }
}

void GLView::enable_headlight()
{
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_NORMALIZE);

    float position[4] = {0.0f,0.0f,0.0f,1.0f};
    glLightfv(GL_LIGHT0,GL_POSITION,position);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE,1);

    float ambient[4] = {0.7f,0.7f,0.7f,1.0f};
    glMaterialfv(GL_FRONT,GL_AMBIENT,ambient);

    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
}



void GLView::draw_gradient(const float* top_color, const float* bot_color)
{
    glPushAttrib(GL_ENABLE_BIT);
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();

    glBegin(GL_QUADS);
    glColor3fv(bot_color);
    glVertex3i(-1,-1,-1);
    glVertex3i(1,-1,-1);
    glColor3fv(top_color);
    glVertex3i(1,1,-1);
    glVertex3i(-1,1,-1);
    glEnd();

    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();

    glPopAttrib();
}

void GLView::draw_xz_grid(int width, int height, float scale)
{
    glPushAttrib(GL_LINE_BIT);

    glLineWidth(2.0f);
    glBegin(GL_LINES);
    glColor3f(1.0f,0.0f,0.0f);
    glVertex3f(-scale*width/2, 0.0f, 0.0f);
    glVertex3f(scale*width/2, 0.0f, 0.0f);
    glColor3f(0.0f,0.0f,1.0f);
    glVertex3f(0.0f, 0.0f, -scale*height/2);
    glVertex3f(0.0f, 0.0f, scale*height/2);
    glEnd();

    glColor3f(0.5f,0.5f,0.5f);
    glLineWidth(1.0f);
    glBegin(GL_LINES);
    for (int i = 0; i <= width; ++i) {
        glVertex3f(scale*(i-width/2), 0.0f, -scale*height/2);
        glVertex3f(scale*(i-width/2), 0.0f, scale*height/2);
    }
    for (int i = 0; i <= height; ++i) {
        glVertex3f(-scale*width/2, 0.0f, scale*(i-height/2));
        glVertex3f(scale*width/2, 0.0f, scale*(i-height/2));
    }
    glEnd();

    glPopAttrib();
}

void GLView::draw_cylinder(float radius, float height, int slices)
{
    for (int i = 0; i < slices; ++i) {
        float alpha = 2*M_PI*i/slices;
        float beta = 2*M_PI*(i+1)/slices;

        float cos_alpha = cos(alpha);
        float sin_alpha = sin(alpha);

        float cos_beta = cos(beta);
        float sin_beta = sin(beta);

        glBegin(GL_TRIANGLES);
        glNormal3f(0,-1,0);
        glVertex3f(radius*cos_alpha,-height/2,radius*sin_alpha);
        glVertex3f(radius*cos_beta,-height/2,radius*sin_beta);
        glVertex3f(0,-height/2,0);
        glEnd();

        glBegin(GL_QUADS);
        glNormal3f(cos_alpha,0,sin_alpha);
        glVertex3f(radius*cos_alpha,-height/2,radius*sin_alpha);
        glVertex3f(radius*cos_alpha,height/2,radius*sin_alpha);
        glVertex3f(radius*cos_beta,height/2,radius*sin_beta);
        glVertex3f(radius*cos_beta,-height/2,radius*sin_beta);
        glEnd();

        glBegin(GL_TRIANGLES);
        glNormal3f(0,1,0);
        glVertex3f(radius*cos_alpha,height/2,radius*sin_alpha);
        glVertex3f(0,height/2,0);
        glVertex3f(radius*cos_beta,height/2,radius*sin_beta);
        glEnd();
    }
}

void GLView::draw_camera()
{
    float length = 1.0f;
    float width = 0.3f;
    float height = 0.5f;

    glBegin(GL_QUADS);

    glNormal3f(0.0f,1.0f,0.0f);
    glVertex3f(-width,height,-length);
    glVertex3f(-width,height,length);
    glVertex3f(width,height,length);
    glVertex3f(width,height,-length);

    glNormal3f(1.0f,0.0f,0.0f);
    glVertex3f(width,-height,-length);
    glVertex3f(width,height,-length);
    glVertex3f(width,height,length);
    glVertex3f(width,-height,length);

    glNormal3f(0.0f,-1.0f,0.0f);
    glVertex3f(-width,-height,-length);
    glVertex3f(width,-height,-length);
    glVertex3f(width,-height,length);
    glVertex3f(-width,-height,length);

    glNormal3f(-1.0f,0.0f,0.0f);
    glVertex3f(-width,-height,-length);
    glVertex3f(-width,-height,length);
    glVertex3f(-width,height,length);
    glVertex3f(-width,height,-length);

    glNormal3f(0.0f,0.0f,1.0f);
    glVertex3f(-width,-height,length);
    glVertex3f(width,-height,length);
    glVertex3f(width,height,length);
    glVertex3f(-width,height,length);

    glNormal3f(0.0f,0.0f,-1.0f);
    glVertex3f(-width,-height,-length);
    glVertex3f(-width,height,-length);
    glVertex3f(width,height,-length);
    glVertex3f(width,-height,-length);

    glEnd();

    glBegin(GL_TRIANGLES);

    float mag = sqrt(height*height+9*length*length/25);

    glNormal3f(3*length/5/mag,0.0f,height/mag);
    glVertex3f(0,0,-length);
    glVertex3f(height,-height,-8*length/5);
    glVertex3f(height,height,-8*length/5);

    glNormal3f(-3*length/5/mag,0.0f,height/mag);
    glVertex3f(0,0,-length);
    glVertex3f(-height,height,-8*length/5);
    glVertex3f(-height,-height,-8*length/5);

    glNormal3f(0.0f,3*length/5/mag,height/mag);
    glVertex3f(0,0,-length);
    glVertex3f(height,height,-8*length/5);
    glVertex3f(-height,height,-8*length/5);

    glNormal3f(0.0f,-3*length/5/mag,height/mag);
    glVertex3f(0,0,-length);
    glVertex3f(-height,-height,-8*length/5);
    glVertex3f(height,-height,-8*length/5);

    glEnd();

    glPushMatrix();
    glTranslatef(0.0f,11*height/5,6*height/5);
    glRotatef(90.0f,0.0f,0.0f,1.0f);
    draw_cylinder(4*height/3,width,10);
    glPopMatrix();

    glPushMatrix();
    glTranslatef(0.0f,11*height/5,-6*height/5);
    glRotatef(90.0f,0.0f,0.0f,1.0f);
    draw_cylinder(4*height/3,width,10);
    glPopMatrix();
}

void GLView::draw_textured_quad(const double* pts, unsigned int texid)
{
    glPushAttrib(GL_ENABLE_BIT);

    //glColor3f(1.0f,1.0f,1.0f);

    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D,texid);
    glTexEnvf(GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE,GL_REPLACE);

    glBegin(GL_QUADS);
    glTexCoord2f(0.0f, 0.0f);
    glVertex3d(pts[0], pts[1],  pts[2]);
    glTexCoord2f(0.0f, 1.0f);
    glVertex3d(pts[3], pts[4],  pts[5]);
    glTexCoord2f(1.0f, 1.0f);
    glVertex3d(pts[6], pts[7],  pts[8]);
    glTexCoord2f(1.0f, 0.0f);
    glVertex3d(pts[9], pts[10], pts[11]);
    glEnd();

    glPopAttrib();
}

void GLView::drawViewport(const CoordFrame& modelview, int volumeId){
	CameraViewWidget * cameraViewWidget = dynamic_cast <CameraViewWidget *> ( this->parent());
	double bb [4];

	cameraViewWidget->getMainWindow()->getTracker()->calculate_viewport(modelview,bb,volumeId);

	glPushAttrib(GL_ENABLE_BIT);
	glColor3f(1.0f,0.0f,0.0f);
	double z = -2;
	glBegin(GL_LINE_LOOP);
	glVertex3d(bb[0], bb[1],  z);
    glVertex3d(bb[0] + bb[2], bb[1],  z);
    glVertex3d(bb[0] + bb[2], bb[1] + bb[3],  z);
    glVertex3d(bb[0], bb[1] + bb[3],  z);
	glEnd();
	glPopAttrib();
}

void GLView::drawBB(const CoordFrame& modelview, int volumeId){
	CameraViewWidget * cameraViewWidget = dynamic_cast <CameraViewWidget *> ( this->parent());

	double corners_[24] = {1,1,1, 1,1,-1, 1,-1,1, -1,1,1, 1,-1,-1, -1,1,-1, -1,-1,1, -1,-1,-1};
	for (int j = 0; j < 8; j++) {
		cameraViewWidget->getMainWindow()->getTracker()->getBBPoint(modelview,&corners_[3*j],volumeId);
	}

	glPushAttrib(GL_ENABLE_BIT);
	glColor3f(0.0f,1.0f,0.0f);
	glBegin(GL_LINES);
	
	glVertex3d(corners_[0], corners_[1], corners_[2]);
	glVertex3d(corners_[3], corners_[4], corners_[5]);

	glVertex3d(corners_[0], corners_[1], corners_[2]);
	glVertex3d(corners_[6], corners_[7], corners_[8]);

	glVertex3d(corners_[0], corners_[1], corners_[2]);
	glVertex3d(corners_[9], corners_[10], corners_[11]);

	glVertex3d(corners_[3], corners_[4], corners_[5]);
	glVertex3d(corners_[12], corners_[13], corners_[14]);

	glVertex3d(corners_[12], corners_[13], corners_[14]);
	glVertex3d(corners_[6], corners_[7], corners_[8]);

	glVertex3d(corners_[6], corners_[7], corners_[8]);
	glVertex3d(corners_[18], corners_[19], corners_[20]);

	glVertex3d(corners_[12], corners_[13], corners_[14]);
	glVertex3d(corners_[21], corners_[22], corners_[23]);

	glVertex3d(corners_[9], corners_[10], corners_[11]);
	glVertex3d(corners_[15], corners_[16], corners_[17]);

	glVertex3d(corners_[3], corners_[4], corners_[5]);
	glVertex3d(corners_[15], corners_[16], corners_[17]);

	glVertex3d(corners_[15], corners_[16], corners_[17]);
	glVertex3d(corners_[21], corners_[22], corners_[23]);

	glVertex3d(corners_[21], corners_[22], corners_[23]);
	glVertex3d(corners_[18], corners_[19], corners_[20]);

	glVertex3d(corners_[18], corners_[19], corners_[20]);
	glVertex3d(corners_[9], corners_[10], corners_[11]);
	glEnd();

}

void GLView::drawLocalCoordAxes(const CoordFrame& modelview, int volumeId){
	CameraViewWidget * cameraViewWidget = dynamic_cast <CameraViewWidget *> ( this->parent());

	double corners2_[21] = {0,0,0, 1,0,0, 0,0,1, 0,1,0, 0,0,-1, 0,-1,0, -1,0,0};
	for (int j = 0; j < 7; j++) {
		cameraViewWidget->getMainWindow()->getTracker()->getBBPoint(modelview,&corners2_[3*j],volumeId);
	}

	glColor3f(1.0f,0.0f,0.0f);
	glBegin(GL_LINES);
	for (int k = 1; k < 7; k++){
		glVertex3d(corners2_[0], corners2_[1], corners2_[2]);
		glVertex3d(corners2_[3* k], corners2_[3* k + 1], corners2_[3* k + 2]);
	}

	glEnd();
	glPopAttrib();
}

void GLView::draw2DProjectionBB(const CoordFrame& modelview, int volumeId){

	CameraViewWidget * cameraViewWidget = dynamic_cast <CameraViewWidget *> ( this->parent());

	double corners_[24] = {1,1,1, 1,1,-1, 1,-1,1, -1,1,1, 1,-1,-1, -1,1,-1, -1,-1,1, -1,-1,-1};
	for (int j = 0; j < 8; j++) {
		cameraViewWidget->getMainWindow()->getTracker()->getBBPoint(modelview,&corners_[3*j],volumeId);
	}

	double corners2_[21] = {0,0,0, 1,0,0, 0,0,1, 0,1,0, 0,0,-1, 0,-1,0, -1,0,0};
	for (int j = 0; j < 7; j++) {
		cameraViewWidget->getMainWindow()->getTracker()->getBBPoint(modelview,&corners2_[3*j],volumeId);
	}

	double min_area = 10000000000;
	double min_angle = 10000000000;

	double final_x = 0.0;
	double final_y = 0.0;
	double final_min_x = 0.0;
	double final_min_y = 0.0;
	double final_max_x = 0.0;
	double final_max_y = 0.0;

	double center_x =  corners2_[0] / corners_[2];
	double center_y =  corners2_[1] / corners_[2];

	for (int i = 0; i < 3; i++){ 
		double curr_z = corners2_[2];
		double curr_x = corners2_[0] / curr_z;
		double curr_y = corners2_[1] / curr_z;
		
		double target_z = corners2_[3*(i+1) + 2];
		double target_x = corners2_[3*(i+1)] / target_z;
		double target_y = corners2_[3*(i+1) + 1] / target_z;

		double vec_x = curr_x - target_x;
		double vec_y = curr_y - target_y;
		double norm = sqrt(pow(vec_x,2) + pow(vec_y,2));
		
		vec_x /= norm;
		vec_y /= norm;

		double angle = acos(vec_y); 

		double rotation_matrix[2][2] = {{cos(angle), -sin(angle)}, {sin(angle), cos(angle)}};

		double corners_project[15]; 
		for (int i = 0; i < 15; i++) corners_project[i] = 0;

		double min_x = 1000000000.0;
		double min_y = 1000000000.0;
		double max_x = 0.0;
		double max_y = 0.0;

		double result_point[8][2];
		for (int j = 0; j < 8; j++){

			double model_x = corners_[3*j] / corners_[3*j+2];
			double model_y = corners_[3*j + 1] / corners_[3*j+2];

			model_x -= center_x;
			model_y -= center_y;

			double temp_point[2] = {model_x, model_y};

			for (int y = 0; y < 2; y++){
				double tempRowSum = 0.0;
				for (int x = 0; x < 2; x++){
					tempRowSum += (rotation_matrix[y][x] * temp_point[x]);
				}
				result_point[i][y] = tempRowSum;
			}
			
			if (result_point[i][0] < min_x){
				min_x = result_point[i][0];
			}
			if (result_point[i][0] > max_x){
				max_x = result_point[i][0];
			}
			if (result_point[i][1] < min_y){
				min_y = result_point[i][1];
			}
			if (result_point[i][1] > max_y){
				max_y = result_point[i][1];
			}

			result_point[i][0] +=center_x;
			result_point[i][1] +=center_y;

		}

			if (i == 0) glColor3f(1.0f,0.0f,0.0f);
			else if (i == 1) glColor3f(1.0f,1.0f,0.0f);
			else if (i == 2) glColor3f(1.0f,0.0f,1.0f);
			glBegin(GL_LINES);


		if (((max_x - min_x) != 0 && (max_y - min_y) != 0) && (max_x - min_x) * (max_y - min_y) < min_area) {
			min_area = (max_x - min_x) * (max_y - min_y);
			min_angle = angle;
			final_x = (max_x - min_x);
			final_y = (max_y - min_y);
			final_min_x = min_x;
			final_min_y = min_y;
			final_max_x = max_x;
			final_max_y = max_y;
		}

	}

	double rotated_pt[4][2];
	rotated_pt[0][0] = final_max_x;
	rotated_pt[0][1] = final_max_y;

	rotated_pt[1][0] = final_min_x;
	rotated_pt[1][1] = final_min_y;

	rotated_pt[2][0] = final_max_x;
	rotated_pt[2][1] = final_min_y;

	rotated_pt[3][0] = final_min_x;
	rotated_pt[3][1] = final_max_y;

	double unrotated_pts[4][2];
	double rotation_matrix[2][2] = {{cos(min_angle), -sin(min_angle)}, {sin(min_angle), cos(min_angle)}};

	for (int sum = 0; sum < 4; sum++){
		for (int i = 0; i < 2; i++){
			double tempRowSums= 0;
			for (int j = 0; j < 2; j++){
				tempRowSums+= (rotation_matrix[j][i] * rotated_pt[sum][j]);
			}
			unrotated_pts[sum][i] = tempRowSums;
		}
	}

	for (int x = 0; x < 4; x++){
		unrotated_pts[x][0] = -2*(unrotated_pts[x][0] +center_x);
		unrotated_pts[x][1] = -2*(unrotated_pts[x][1] +center_y);
	}

	glVertex3d(unrotated_pts[0][0],unrotated_pts[0][1],-2);
	glVertex3d(unrotated_pts[2][0],unrotated_pts[2][1],-2);

	glVertex3d(unrotated_pts[1][0],unrotated_pts[1][1],-2);
	glVertex3d(unrotated_pts[2][0],unrotated_pts[2][1],-2);

	glVertex3d(unrotated_pts[0][0],unrotated_pts[0][1],-2);
	glVertex3d(unrotated_pts[3][0],unrotated_pts[3][1],-2);

	glVertex3d(unrotated_pts[1][0],unrotated_pts[1][1],-2);
	glVertex3d(unrotated_pts[3][0],unrotated_pts[3][1],-2);
			
	glEnd();
	glPopAttrib();
}