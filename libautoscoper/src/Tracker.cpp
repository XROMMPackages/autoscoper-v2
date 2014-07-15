// ----------------------------------
// Copyright (c) 2011, Brown University
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// (1) Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//
// (2) Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// (3) Neither the name of Brown University nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY BROWN UNIVERSITY “AS IS” WITH NO
// WARRANTIES OR REPRESENTATIONS OF ANY KIND WHATSOEVER EITHER EXPRESS OR
// IMPLIED, INCLUDING WITHOUT LIMITATION ANY WARRANTY OF DESIGN OR
// MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE, EACH OF WHICH ARE
// SPECIFICALLY DISCLAIMED, NOR ANY WARRANTY OR REPRESENTATIONS THAT THE
// SOFTWARE IS ERROR FREE OR THAT THE SOFTWARE WILL NOT INFRINGE ANY
// PATENT, COPYRIGHT, TRADEMARK, OR OTHER THIRD PARTY PROPRIETARY RIGHTS.
// IN NO EVENT SHALL BROWN UNIVERSITY BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
// OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY OR CAUSE OF ACTION, WHETHER IN CONTRACT,
// STRICT LIABILITY, TORT, NEGLIGENCE OR OTHERWISE, ARISING IN ANY WAY
// OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
// SUCH DAMAGE. ANY RECIPIENT OR USER OF THIS SOFTWARE ACKNOWLEDGES THE
// FOREGOING, AND ACCEPTS ALL RISKS AND LIABILITIES THAT MAY ARISE FROM
// THEIR USE OF THE SOFTWARE.
// ---------------------------------

/// \file Tracker.hpp
/// \author Andy Loomis

#include "Tracker.hpp"

#include <algorithm>
#include <dirent.h>
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifdef WITH_CUDA
#include "gpu/cuda/CudaWrap.hpp"
#include "gpu/cuda/Ncc_kernels.h"
#else
#include "gpu/opencl/Ncc.hpp"
#endif

#ifdef WITH_CMINPACK
#include "cminpack.h"
#endif
 
#define DIM_PER_OBJ 6
#define sqr(a) ((a) * (a))

#include "VolumeDescription.hpp"
#include "Video.hpp"
#include "View.hpp"
#include "DownhillSimplex.hpp"
#include "Camera.hpp"
#include "CoordFrame.hpp"


using namespace std;

static bool firstRun = true;

// XXX
// Set callback for Downhill Simplex. This is really a hack so that we can use
// define the global function pointer FUNC, which Downhill SImplex will
// optimize.

static xromm::Tracker* g_markerless = NULL;
double FUNC(double* P) { return g_markerless->minimizationFunc(P+1); }

// pointer to cminpack function
#ifdef WITH_CMINPACK
	int (*FCN)(void *p, int m, int n, const double *x, double *FVEC, int iflag);
#endif

namespace xromm {

#if DEBUG
void save_debug_image(const gpu::Buffer* dev_image, int width, int height)
{
	static int count = 0;
	float* host_image = new float[width*height];
	unsigned char* uchar_image = new unsigned char[width*height];

	// Copy the image to the host
	dev_image->write(host_image, width*height*sizeof(float));
	//cudaMemcpy(host_image,dev_image,width*height*sizeof(float),cudaMemcpyDeviceToHost);

	// Copy to a char array
	for (int i = 0; i < width*height; i++) {
		uchar_image[i] = (int)(255*host_image[i]);
	}

	char filename[256];
	sprintf(filename,"image_%02d.ppm",count++);
	ofstream file(filename,ios::out);
	file << "P2" << endl;
	file << width << " " << height << endl;
	file << 255 << endl;
	for (int i = 0; i < width*height; i++) {
		file << (int)uchar_image[i] << " ";
	}

	delete[] uchar_image;
	delete[] host_image;
}
#endif

Tracker::Tracker()
    : rendered_drr_(NULL),
      rendered_rad_(NULL)
{
    g_markerless = this;
}

Tracker::~Tracker()
{
	for (int i = 0 ;i < volumeDescription_.size(); i++){
		delete volumeDescription_[i];
	}
	volumeDescription_.clear();
}

void Tracker::init()
{
#ifdef WITH_CUDA
    gpu::cudaInitWrap();
#endif
}

void Tracker::load(const Trial& trial)
{
    trial_ = trial;

    vector<gpu::View*>::iterator viewIter;
    for (viewIter = views_.begin(); viewIter != views_.end(); ++viewIter) {
        delete *viewIter;
    }
    views_.clear();

    for (int i = 0 ;i < volumeDescription_.size(); i++){
		delete volumeDescription_[i];
	}
	volumeDescription_.clear();
	for (int i = 0 ;i < trial_.volumes.size(); i++){
		volumeDescription_.push_back(new gpu::VolumeDescription(trial_.volumes[i]));
	}
    

	unsigned npixels = trial_.render_width*trial_.render_height;
#ifdef WITH_CUDA
	gpu::cudaMallocWrap(rendered_drr_,trial_.render_width*trial_.render_height*sizeof(float));
    gpu::cudaMallocWrap(rendered_rad_,trial_.render_width*trial_.render_height*sizeof(float));
#else
	rendered_drr_ = new gpu::Buffer(npixels*sizeof(float));
	rendered_rad_ = new gpu::Buffer(npixels*sizeof(float));
#endif

    gpu::ncc_init(npixels);

    for (unsigned int i = 0; i < trial_.cameras.size(); ++i) {

        Camera& camera = trial_.cameras.at(i);
        Video& video  = trial_.videos.at(i);

        video.set_frame(trial_.frame);

        gpu::View* view = new gpu::View(camera);

		for(int i = 0 ; i < volumeDescription_.size(); i ++){
			view->addDrrRenderer();
			view->drrRenderer(i)->setVolume(*volumeDescription_[i]);
		}
        view->radRenderer()->set_image_plane(camera.viewport()[0],
                                             camera.viewport()[1],
                                             camera.viewport()[2],
                                             camera.viewport()[3]);
        view->radRenderer()->set_rad(video.data(),
                                     video.width(),
                                     video.height(),
                                     video.bps());

        views_.push_back(view);
    }
}

int levmar_minimizationFunc(void *p, int mFuncs, int nVars, const double *values, double *FVEC, int iflag);

void Tracker::optimize(int frame, int dFrame, int repeats)
{

    if (frame < 0 || frame >= trial_.num_frames) {
        cerr << "Tracker::optimize(): Invalid frame." << endl;
        return;
    }

	//default values
	this->FTOL = 0.01; // default downhill simplex tolerance
	this->lTolerance = 0.01; // default levenberg marquardt tolerance
	this->box_division_factor = 1; // default one bounding box

    int NDIM =  6 * trial_.num_volumes;       // Number of dimensions to optimize over.
    //double FTOL = 0.01; // Tolerance for the optimization. -- still kept at default value
	double LTOL = sqrt(__cminpack_func__(dpmpar)(1));
    MAT P;              // Matrix of points to initialize the routine.
    double Y[MP];       // The values of the minimization function at the
                        // initial points.
    int ITER;

    trial_.frame = frame;

    for (unsigned int i = 0; i < trial_.videos.size(); ++i) {

        trial_.videos.at(i).set_frame(trial_.frame);
        views_[i]->radRenderer()->set_rad(trial_.videos.at(i).data(),
                                          trial_.videos.at(i).width(),
                                          trial_.videos.at(i).height(),
                                          trial_.videos.at(i).bps());
    }

	for (int i = 0; i < trial_.num_volumes; i++){
		int framesBehind = (dFrame > 0)?
					   (int)trial_.frame:
                       (int)trial_.num_frames-trial_.frame-1;
		if (trial_.guess == 2 && framesBehind > 1) {
			double xyzypr1[6] = { (*trial_.getXCurve(i))(trial_.frame-2*dFrame),
				                  (*trial_.getYCurve(i))(trial_.frame-2*dFrame),
					              (*trial_.getZCurve(i))(trial_.frame-2*dFrame),
						          (*trial_.getYawCurve(i))(trial_.frame-2*dFrame),
							      (*trial_.getPitchCurve(i))(trial_.frame-2*dFrame),
								  (*trial_.getRollCurve(i))(trial_.frame-2*dFrame) };
			double xyzypr2[6] = { (*trial_.getXCurve(i))(trial_.frame-dFrame),
				                  (*trial_.getYCurve(i))(trial_.frame-dFrame),
					              (*trial_.getZCurve(i))(trial_.frame-dFrame),
						          (*trial_.getYawCurve(i))(trial_.frame-dFrame),
							      (*trial_.getPitchCurve(i))(trial_.frame-dFrame),
								  (*trial_.getRollCurve(i))(trial_.frame-dFrame) };

	       CoordFrame xcframe = CoordFrame::from_xyzypr(xyzypr1).linear_extrap(
		                         CoordFrame::from_xyzypr(xyzypr2));

	        xcframe.to_xyzypr(xyzypr1);
			trial_.getXCurve(i)->insert(trial_.frame,xyzypr1[0]);
			trial_.getYCurve(i)->insert(trial_.frame,xyzypr1[1]);
			trial_.getZCurve(i)->insert(trial_.frame,xyzypr1[2]);
		    trial_.getYawCurve(i)->insert(trial_.frame,xyzypr1[3]);
		    trial_.getPitchCurve(i)->insert(trial_.frame,xyzypr1[4]);
		    trial_.getRollCurve(i)->insert(trial_.frame,xyzypr1[5]); 
		}
		else if (trial_.guess == 1 && framesBehind > 0) {
		    trial_.getXCurve(i)->insert(trial_.frame, (*trial_.getXCurve(i))(trial_.frame-dFrame));
			trial_.getYCurve(i)->insert(trial_.frame,  (*trial_.getYCurve(i))(trial_.frame-dFrame));
			trial_.getZCurve(i)->insert(trial_.frame,  (*trial_.getZCurve(i))(trial_.frame-dFrame));
			trial_.getYawCurve(i)->insert(trial_.frame,  (*trial_.getYawCurve(i))(trial_.frame-dFrame));
			trial_.getPitchCurve(i)->insert(trial_.frame, (*trial_.getPitchCurve(i))(trial_.frame-dFrame));
			trial_.getRollCurve(i)->insert(trial_.frame,  (*trial_.getRollCurve(i))(trial_.frame-dFrame));
		}
	}

    int totalIter = 0;
    for (int j = 0; j < repeats; j++) {

		//fill initial vectors
		for (int i = 0; i < NDIM + 1; ++i) {
           for (int j = 1; j < NDIM + 1; ++j) {
				P[i+1][j] = (i == j)? trial_.offsets[(j-1) % 6]: 0.0;
		   }
		}

		double *initGuess;

        // Determine the function values at the vertices of the initial
        // simplex
		for (int i = 0; i < NDIM + 1; ++i) {
			Y[i+1] = FUNC(P[i+1]);
		}

        // Optimize the frame
        ITER = 0;

        if (this->trial()->getOptAlg() == DOWNHILL_SIMPLEX){
			AMOEBA(P, Y, NDIM, this->FTOL, &ITER);
		} else if (this->trial()->getOptAlg() == LEVENBERG_MARQUARDT){
#ifdef WITH_CMINPACK

			unsigned numFunctions = trial_.num_volumes * views_.size() * box_division_factor * box_division_factor;
			unsigned numParams = trial_.num_volumes * DIM_PER_OBJ;

			double *initGuess = new double[numParams];
			double *FVEC = 	new double[numFunctions];

			for (int i = 0; i < numParams; i++) initGuess[i] = 0.0;

			int * iwa = new int[numParams];
			int lwa = numFunctions * numParams + 5 * numParams + numFunctions;
			double * wa = new double[lwa];

			int info = __cminpack_func__(lmdif1)(levmar_minimizationFunc, this, numFunctions, numParams, initGuess, FVEC, LTOL, iwa, wa, lwa); 

			for (int i = 0; i < numParams; i++) initGuess[i] *= lTolerance;
#else
			fprintf(stderr, "Error: Optimization: Not Compiled With CMINPACK, Using Downhill Simplex");
			AMOEBA(P, Y, NDIM, this->FTOL, &ITER);
#endif
		}

		for (int i = 0; i < trial_.num_volumes; i++){

			double xyzypr[6] = { (*trial_.getXCurve(i))(trial_.frame),
                             (*trial_.getYCurve(i))(trial_.frame),
                             (*trial_.getZCurve(i))(trial_.frame),
                             (*trial_.getYawCurve(i))(trial_.frame),
                             (*trial_.getPitchCurve(i))(trial_.frame),
                             (*trial_.getRollCurve(i))(trial_.frame) };

			CoordFrame xcframe = CoordFrame::from_xyzypr(xyzypr);
			CoordFrame manip;

			// initiate rotation based on trial specs
			if (this->trial()->getRotationMode() == AXIS_ANGLE) {
				if (this->trial()->getOptAlg() == DOWNHILL_SIMPLEX)
					manip = CoordFrame::from_xyzAxis_angle(&P[1][1  + i * 6]);
				else if (this->trial()->getOptAlg() == LEVENBERG_MARQUARDT)
					manip = CoordFrame::from_xyzAxis_angle(&initGuess[i*6]);
			} else if (this->trial()->getRotationMode() == QUATERNION) {
				if (this->trial()->getOptAlg() == DOWNHILL_SIMPLEX)
					manip = CoordFrame::from_xyzAxis_angle(&P[1][1  + i * 6]);
				else if (this->trial()->getOptAlg() == LEVENBERG_MARQUARDT)
					manip = CoordFrame::from_xyzAxis_angle(&initGuess[i*6]);
			} else if (this->trial()->getRotationMode() == EULER_ANGLE){
				if (this->trial()->getOptAlg() == DOWNHILL_SIMPLEX)
					manip = CoordFrame::from_xyzAxis_angle(&P[1][1  + i * 6]);
				else if (this->trial()->getOptAlg() == LEVENBERG_MARQUARDT)
					manip = CoordFrame::from_xyzAxis_angle(&initGuess[i*6]);
			}

			xcframe = xcframe * trial_.getVolumeMatrix(i)->inverse() * manip * *trial_.getVolumeMatrix(i);

			xcframe.to_xyzypr(xyzypr);
		
			trial_.getXCurve(i)->insert(trial_.frame,xyzypr[0]);
			trial_.getYCurve(i)->insert(trial_.frame,xyzypr[1]);
			trial_.getZCurve(i)->insert(trial_.frame,xyzypr[2]);
			trial_.getYawCurve(i)->insert(trial_.frame,xyzypr[3]);
			trial_.getPitchCurve(i)->insert(trial_.frame,xyzypr[4]);
			trial_.getRollCurve(i)->insert(trial_.frame,xyzypr[5]);


		}

		totalIter += ITER;
	}
		cerr << "Tracker::optimize(): Frame " << trial_.frame
         << " done in " << totalIter << " total iterations" << endl;
}

/* function to compute the viewport of the current bounding box */
void Tracker::computeTempViewport(double *viewport, int viewID, int volID){
	// Calculate the viewport surrounding the volume
	
	if (this->rMode == C){
		double min_max[4] = {1.0,1.0,-1.0,-1.0};
		for (int c = 0; c < trial_.num_volumes; c++){
			this->calculate_viewport(views_[viewID]->drrRenderer(c)->getModelView(),viewport, c);
			//compare min_max with viewport
			if (min_max[0] > viewport[0])
				min_max[0] = viewport[0];
			if (min_max[1] > viewport[1])
				min_max[1] = viewport[1];
			if (min_max[2] < viewport[0] + viewport[2])
				min_max[2] = viewport[0] + viewport[2];
			if (min_max[3] < viewport[1] + viewport[3])
				min_max[3] = viewport[1] + viewport[3];
		}
		//set viewport from min_max
		viewport[0] = min_max[0];
		viewport[1] = min_max[1];
		viewport[2] = min_max[2]-min_max[0];
		viewport[3] = min_max[3]-min_max[1];
	} else {
		this->calculate_viewport(views_[viewID]->drrRenderer(volID)->getModelView(),viewport, volID);
	}
}


/* function used for the lmdir cminpack routine*/
int levmar_minimizationFunc(void *p, int mFuncs, int nVars, const double *values, double *FVEC, int iflag)
{
	double *values_ = new double[nVars];
	Tracker * tracker = (Tracker * ) p;
	for (int i = 0; i < nVars; i++) values_[i] = values[i] * tracker->lTolerance;

	/*FILE * tmp = fopen("tmp.txt","a");
	for(int k = 0; k < nVars; k++)
		fprintf(tmp,"%e ",values_[k]);
	fprintf(tmp,"\n");
	fclose(tmp);*/

	const int view_number = tracker->views().size();

	for (int i = 0; i < tracker->trial()->num_volumes; i++){
		tracker->minFuncCombined(values);
		int rVolumes = tracker->getRenderMode() == C ? 1 : tracker->trial()->num_volumes;
		int insertIndex = 0;

		for (unsigned j = 0; j < tracker->views().size(); j++){
			for (int i = 0; i < rVolumes; i++){
				// Calculate the viewport surrounding the volume
				double viewport[4]; 
				tracker->computeTempViewport(viewport,j,i);
				// compute sub-box width and height
				unsigned trunc_width = viewport[2] / tracker->box_division_factor;
				unsigned trunc_height = viewport[3] / tracker->box_division_factor;
				
				for (int y_offset = 0; y_offset < tracker->box_division_factor; y_offset++){
					for (int x_offset = 0; x_offset < tracker->box_division_factor; x_offset++){

						int x_min = viewport[0] + trunc_width * x_offset;
						int y_min = viewport[1] + trunc_height * y_offset;

						FVEC[insertIndex++] = tracker->getCorrelationScore(viewport,i,j);

					}
				}
			}
		}
	}
	delete [] values_;
	return 0;
}

/* computes the manip coordframe and sets the 
   modelview for the drr and rad renderrer. This
   operation is independent of the optimization used. */
void Tracker::minFuncCombined(const double *values){
	for (int i = 0; i < this->trial()->num_volumes; i++){
		double xyzypr[6] = { (*(const_cast<Trial&>(trial_)).getXCurve(i))(trial_.frame),
                         (*(const_cast<Trial&>(trial_)).getYCurve(i))(trial_.frame),
                         (*(const_cast<Trial&>(trial_)).getZCurve(i))(trial_.frame),
                         (*(const_cast<Trial&>(trial_)).getYawCurve(i))(trial_.frame),
                         (*(const_cast<Trial&>(trial_)).getPitchCurve(i))(trial_.frame),
                         (*(const_cast<Trial&>(trial_)).getRollCurve(i))(trial_.frame)};
		CoordFrame xcframe = CoordFrame::from_xyzypr(xyzypr);
		CoordFrame manip;

		if (rMode == AXIS_ANGLE){
			manip = CoordFrame::from_xyzAxis_angle(&values[6*i]);
		} else if (rMode == QUATERNION){
			manip = CoordFrame::from_xyzijk(&values[6*i]);
		} else if (rMode == EULER_ANGLE){
			manip = CoordFrame::from_xyzypr(&values[6*i]);
		}

		xcframe = xcframe * trial_.getVolumeMatrix(i)->inverse() * manip * *trial_.getVolumeMatrix(i);

		for (unsigned int j = 0; j < views_.size(); ++j) {
			CoordFrame modelview = views_[j]->camera()->coord_frame().inverse()*xcframe;
			double imv[16]; 
			modelview.inverse().to_matrix_row_order(imv);
			views_[j]->drrRenderer(i)->setInvModelView(imv);
			views_[j]->drrRenderer(i)->setModelView(modelview); // set model view and inv model view
		}
	}
}


/* function used to compute the correlation score for the volID object from 
   view number viewNum */
double Tracker::getCorrelationScore(double * viewport, int volID, int viewNum)
{

	unsigned int render_width =   (viewport[2] / views_[viewNum]->camera()->viewport()[2]) * ((float) trial_.render_width);
	unsigned int render_height =  (viewport[3] / views_[viewNum]->camera()->viewport()[3]) * ((float) trial_.render_height);

	float render_width_f =   (viewport[2] / views_[viewNum]->camera()->viewport()[2]) * ((float) trial_.render_width);
	float render_height_f =  (viewport[3] / views_[viewNum]->camera()->viewport()[3]) * ((float) trial_.render_height);

	if (this->rMode == A){
		// Render the DRR and Radiograph
		views_[viewNum]->drrRenderer(volID)->setViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
		views_[viewNum]->renderDrr(rendered_drr_,render_width, render_height,volID);
	} else if (this->rMode == B || this->rMode == C){
		for (int v = 0; v < trial_.num_volumes; v++){
			views_[viewNum]->drrRenderer(v)->setViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
			views_[viewNum]->renderDrr(rendered_drr_,render_width, render_height);
		}
	}

	views_[viewNum]->radRenderer()->set_viewport(viewport[0], viewport[1], viewport[2], viewport[3]);
	views_[viewNum]->renderRad(rendered_rad_,render_width,render_height);

	return 1.0 - gpu::ncc(rendered_drr_, rendered_rad_, render_width*render_height);
}


/* function is used for the downhill simplex calculation */
double Tracker::minimizationFunc(const double* values)
{	
	this->minFuncCombined(values);
	double **object_correlations = new double*[trial_.num_volumes];
	
	for (int i = 0; i < trial_.num_volumes; i++)
		object_correlations[i] = new double[views_.size()];
	
	int rVolumes = (this->rMode == C)? 1 : trial_.num_volumes;

	for (unsigned int j = 0; j < views_.size(); ++j) {
		for (int i = 0; i < rVolumes; i++){
			// Calculate the viewport surrounding the volume
			double viewport[4];
			this->computeTempViewport(viewport, j,i);
			
			// Calculate the size of the image to render
			unsigned render_width = viewport[2] * trial_.render_width / views_[j]->camera()->viewport()[2];
			unsigned render_height = viewport[3] * trial_.render_height / views_[j]->camera()->viewport()[3];

			object_correlations[i][j] = this->getCorrelationScore(viewport,i,j);

#if DEBUG
        save_debug_image(rendered_drr_,render_width,render_height);
        save_debug_image(rendered_rad_,render_width,render_height);
#endif
		}
	}

	double *final_correlations = new double[rVolumes];
	for (unsigned int j = 0; j <rVolumes; j++){
		final_correlations[j] = 0.0 ;
		for (unsigned int i = 0; i < trial_.cameras.size(); ++i)
			if (this->trial()->getComputeCorrelations() == ADD)
				final_correlations[j] += object_correlations[j][i];
			else if (this->trial()->getComputeCorrelations() == MULTIPLY)
				final_correlations[j] *= object_correlations[j][i];
	}
	
	double final_score = (this->trial()->getComputeCorrelations() == ADD) ? 0.0 : 1.0;
	for (unsigned int i = 0; i <rVolumes; i++)
		if (this->trial()->getComputeCorrelations() == ADD)
			final_score += final_correlations[i];
		else if (this->trial()->getComputeCorrelations() == MULTIPLY)
			final_score *= final_correlations[i];

	for (int i = 0; i < rVolumes; i++)
		delete[] object_correlations[i];

	delete[] object_correlations;
	delete[] final_correlations;
	return final_score;
	
}


void
Tracker::calculate_viewport(const CoordFrame& modelview,double* viewport, int volumeId) const
{
    // Calculate the minimum and maximum values of the bounding box
    // corners after they have been projected onto the view plane
    double min_max[4] = {1.0,1.0,-1.0,-1.0};
    double corners[24] = {0,0,-1,0,0,0, 0,1,-1,0,1,0, 1,0,-1,1,0,0,1,1,-1,1,1,0};
	
    for (int j = 0; j < 8; j++) {

        // Calculate the loaction of the corner in object space
        corners[3*j+0] = (corners[3*j+0]-volumeDescription_[volumeId]->invTrans()[0])/volumeDescription_[volumeId]->invScale()[0];
        corners[3*j+1] = (corners[3*j+1]-volumeDescription_[volumeId]->invTrans()[1])/volumeDescription_[volumeId]->invScale()[1];
        corners[3*j+2] = (corners[3*j+2]-volumeDescription_[volumeId]->invTrans()[2])/volumeDescription_[volumeId]->invScale()[2];

        // Calculate the location of the corner in camera space
        double corner[3];
        modelview.point_to_world_space(&corners[3*j],corner);

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

    viewport[0] = min_max[0];
    viewport[1] = min_max[1];
    viewport[2] = min_max[2]-min_max[0];
    viewport[3] = min_max[3]-min_max[1];
}

} // namespace XROMM
