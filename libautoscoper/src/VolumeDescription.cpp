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

/// \file VolumeDescription.cpp
/// \author Andy Loomis, Mark Howison

#include <opencv\cv.hpp>

#include <iostream>
#include <limits>
#include <fstream>
#include <vector>

#ifdef WITH_CUDA
#include <cuda.h>
#include <cutil_inline.h>
#include <cutil_math.h>
#endif

#include "Volume.hpp"
#include "VolumeDescription.hpp"

#undef max
#undef min

using namespace std;
using namespace cv;

template <class T>
void copyVolume(T* dest,
                const T* src,
                int width,
                int height,
                int depth,
                const int* min,
                const int* max,
                T* minVal,
                T* maxVal)
{
    *minVal = numeric_limits<T>::max();
    *maxVal = numeric_limits<T>::min();
    for (int k = min[2]; k < max[2]+1; k++) {
        for (int i = min[1]; i < max[1]+1; i++) {
            for (int j = min[0]; j < max[0]+1; j++) {
                if (src[k*width*height+i*width+j] < *minVal) {
                    *minVal = src[k*width*height+i*width+j];
                }
                if (src[k*width*height+i*width+j] > *maxVal) {
                    *maxVal = src[k*width*height+i*width+j];
                }
                *dest++ = src[k*width*height+i*width+j];
            }
        }
    }
}

namespace xromm { namespace gpu
{


template <class T> void VolumeDescription::cropVolume(const T* data,
                int width,
                int height,
                int depth,
                int* min,
                int* max
		)
{

	cv::Mat X;
	int numValidPoints = 0;

    min[0] = width;
    min[1] = height;
    min[2] = depth;
    max[0] = 0;
    max[1] = 0;
    max[2] = 0;
    const T* dp1_ = data;

	// determine the number of points to 
	// prevent out_of_memory error
	for (int k_ = 0; k_ < depth; k_++){
		for (int i_ = 0; i_ < height; i_++){
			for (int j_ = 0; j_ < width; j_++){
				if (*dp1_++ != T(0)) {
					numValidPoints++;
				}
			}
		}
	}

	// create the matrix
	X.create(numValidPoints,3,CV_64F);
	int numValidPointsToInsert = 0;
	const T* dp1 = data;

    for (int k = 0; k < depth; k++) {
		bool nonZeroCol = false;
        for (int i = 0; i < height; i++) {
			bool nonZeroRow = false;
            for (int j = 0; j < width; j++) {
                if (*dp1++ != T(0)) {
					X.at<double>(numValidPointsToInsert,0) = (double) j;
					X.at<double>(numValidPointsToInsert,1) = (double) i;
					X.at<double>(numValidPointsToInsert++,2) = (double) k;
                    if (j < min[0]) {
                        min[0] = j;
                    }
                    if (j > max[0]) {
                        max[0] = j;
                    }
                    nonZeroRow = true;
                }
            }
            if (nonZeroRow) {
                if (i < min[1]) {
                    min[1] = i;
                }
                if (i > max[1]) {
                    max[1] = i;
                }
                nonZeroCol = true;
            }
        }
        if (nonZeroCol) {
            if (k < min[2]) {
                min[2] = k;
            }
            if (k > max[2]) {
                max[2] = k;
            }
        }
    }

	
	// perform PCA on cropped matrix
	cv::PCA resultsPCA = cv::PCA::PCA(X, Mat(), CV_PCA_DATA_AS_ROW, 3);
	cv::Mat eigenVectors, mean;
	mean = resultsPCA.mean;
	eigenVectors = resultsPCA.eigenvectors;
	cv::Mat transX;
	cv::transpose(X,transX);
	cv::Scalar mX = cv::mean(transX.col(0));
	mX[0] = mean.at<double>(0);
	mX[1] = mean.at<double>(1);
	mX[2] = mean.at<double>(2);
	
	// go from global to local coordinates / P is local coordinates of X
	cv::Mat P = resultsPCA.project(X);

	double pMIN[3], pMAX[3];
	for (int k = 0; k < 3; k++){
		cv::Mat c = P.col(k); // get the kth column of P
		cv::minMaxLoc(c,&pMIN[k],&pMAX[k]);
	}

	double localCenter[3];
	for (int i = 0; i < 3; i++)
		localCenter[i] = (pMIN[i] + pMAX[i])/2; // the center of the local coordinate system (P)

	double localscale[4];
	for (int i = 0; i < 3; i++)	
		localscale[i] = (pMAX[i] - pMIN[i])/2; // scaling from unitcoordinates (-1 , 1) to local coordinats (min , max) 
	localscale[3] = 1;

	
	// these are dynamically allocated so they can be saved

	/* --------- set up translate/rotate --------- */
	for (int i = 0; i < eigenVectors.cols; i++){
		for (int j = 0; j < eigenVectors.rows; j++){
			globalToMinMaxLocal_[i][j] = eigenVectors.at<double>(j,i);
		}
	}

	for (int p = 0; p < 3; p++){
		globalToMinMaxLocal_[p][3] = mX[p];
		globalToMinMaxLocal_[3][p] = 0;
	}
	globalToMinMaxLocal_[eigenVectors.rows][eigenVectors.cols] = 1; // bottom right hand corner element

	/* --------- set up PCA mean correction --------- */
	for (int i = 0; i < 3; i++){
		for (int j = 0; j < 3; j++){
			if (i == j) 
				minMaxLocalCorrection_[i][j] = 1;
			else minMaxLocalCorrection_[i][j] = 0;
		}
	}
	for (int k = 0; k < 3; k++){
		minMaxLocalCorrection_[3][k] = -localCenter[k];
		minMaxLocalCorrection_[k][3] = 0;
	}

	minMaxLocalCorrection_[3][3] = 1;

	/* --------- set up scale correction matrix --------- */
	for (int s = 0; s < 4; s++){
		for (int _s = 0; _s < 4; _s++){
			if (s == _s) {
				scaleCorrectionMatrix_[s][_s] = localscale[s];
			}else{
				scaleCorrectionMatrix_[s][_s] = 0;
			}
		}
	}	

/* X <-> P corresondance test
	for (int i = 0; i < 10; i++) { // for all rows in x
		double *currTestPoint = new double[4];
		for (int copy = 0; copy < 3; copy++)  currTestPoint[copy] = P.at<double>(i,copy);
		currTestPoint[3] = 1;
		double globalToMinLocalTranspose[4][4];
	
		// compute the transpose of the matrix attained from projection
		for (int y = 0; y < 4; y++){
			for (int x = 0; x < 4; x++){
				globalToMinLocalTranspose[x][y] = globalToMinMaxLocal_[y][x];
			}
		}

		double *finalResult = new double[4];

		// multiply the transpose matrix by the currTestPoint in P.
		for (int i = 0; i < 4; i++){
			double tempRowSum = 0.0;
			for (int j = 0; j < 4; j++){
				tempRowSum += (globalToMinMaxLocal_[i][j] * currTestPoint[j]);
			}
			finalResult[i] = tempRowSum;
		}

		// get the corresponding point in X for comparison
		double *currComparePoint = new double[4];
		for (int get = 0; get < 3; get++) currComparePoint[get] = X.at<double>(i,get); 
		currComparePoint[3] = 1;
		// compare the original point in X to the point attained by multiplying 
		// the point from P by the transpose. 
		for (int i = 0; i < 4; i++)
			
			(stderr, "finalResult[%d] = %lf, currComparPoint[%d] = %lf\n", i, finalResult[i], i, currComparePoint[i]);
	}
*/

}

VolumeDescription::VolumeDescription(const Volume& volume)
    : minValue_(0.0f), maxValue_(1.0f), image_(0)
{
    // Crop the volume
    int min[3], max[3];
    switch(volume.bps()) {
        case 8: {
            cropVolume((unsigned char*)volume.data(),
                    (int)volume.width(),
                    (int)volume.height(),
                    (int)volume.depth(),
                    min,
                    max);
            break;
        }
        case 16: {
            cropVolume((unsigned short*)volume.data(),
                    (int)volume.width(),
                    (int)volume.height(),
                    (int)volume.depth(),
                    min,
                    max);
            break;
        }
        default: {
            cerr << "VolumeDescription(): Unsupported bit-depth "
                                          << volume.bps() << endl;
            exit(0);
        }
    }
	// after the crop volume function is called,
	// global_mat_a, b, and c are set in the crop volume
	// function. We can safely assign these to an instance 
	// of the volume discription.

    // The volume is empty
    if (min[0] > max[0] || min[1] > max[1] || min[2] > max[2]) {
        std::cerr << "Empty Volume" << std::endl;
        exit(0);
    }

    // Copy to the cropped volume
    int dim[3] = { max[0]-min[0]+1, max[1]-min[1]+1, max[2]-min[2]+1 };
    vector<char> data(dim[0]*dim[1]*dim[2]*(volume.bps()/8));
    switch(volume.bps()) {
        case 8: {
            unsigned char minVal, maxVal;
            copyVolume((unsigned char*)&data[0],
                    (unsigned char*)volume.data(),
                    (int)volume.width(),
                    (int)volume.height(),
                    (int)volume.depth(),
                    min,
                    max,
                    &minVal,
                    &maxVal);
            minValue_ = minVal/(float)numeric_limits<unsigned char>::max();
            maxValue_ = maxVal/(float)numeric_limits<unsigned char>::max();
            break;
        }
        case 16: {
            unsigned short minVal, maxVal;
            copyVolume((unsigned short*)&data[0],
                    (unsigned short*)volume.data(),
                    (int)volume.width(),
                    (int)volume.height(),
                    (int)volume.depth(),
                    min,
                    max,
                    &minVal,
                    &maxVal);
            minValue_ = minVal/(float)numeric_limits<unsigned short>::max();
            maxValue_ = maxVal/(float)numeric_limits<unsigned short>::max();
            break;
        }
        default:
            cerr << "VolumeDescription(): Unsupported bit-depth "
                                          << volume.bps() << endl;
            exit(0);
    }

	scale_[0] = volume.scaleX();
    scale_[1] = volume.scaleY();
    scale_[2] = volume.scaleZ();
	height_ = volume.height();

    // Calculate the offset and size of the sub-volume
    invScale_[0] = 1.0f/(float)(volume.scaleX()*dim[0]);
    invScale_[1] = 1.0f/(float)(volume.scaleY()*dim[1]);
    invScale_[2] = 1.0f/(float)(volume.scaleZ()*dim[2]);

    invTrans_[0] = -min[0]/(float)dim[0];
    invTrans_[1] = -((volume.height()-max[1]-1)/(float)dim[1]);
    invTrans_[2] = min[2]/(float)dim[2];

    flips_[0] = volume.flipX();
    flips_[1] = volume.flipY();
    flips_[2] = volume.flipZ();


    // Free any previously allocated memory.
	
#ifdef WITH_CUDA
	// Free any previously allocated memory.
    cutilSafeCall(cudaFreeArray(image_));

    // Create a 3D array.
    cudaChannelFormatDesc desc;
    switch(volume.bps()) {
        case 8: desc = cudaCreateChannelDesc<unsigned char>(); break;
        case 16: desc = cudaCreateChannelDesc<unsigned short>(); break;
        default:
                 cerr << "VolumeDescription(): Unsupported bit-depth "
                                               << volume.bps() << endl;
                 exit(0);
    }
    cudaExtent extent = make_cudaExtent(dim[0], dim[1], dim[2]);
    cutilSafeCall(cudaMalloc3DArray(&image_, &desc, extent));

    // Copy volume to 3D array.
    cudaMemcpy3DParms copyParams = {0};
    copyParams.srcPtr = make_cudaPitchedPtr(&data[0],
            extent.width*(volume.bps()/8),
            extent.width, extent.height);
    copyParams.dstArray = image_;
    copyParams.extent = extent;
    copyParams.kind = cudaMemcpyHostToDevice;
    cutilSafeCall(cudaMemcpy3D(&copyParams));
#else
	if (image_) delete image_;

    // Create a 3D array.
	cl_image_format format;
	format.image_channel_order = CL_R;
    switch (volume.bps()) {
        case 8:  format.image_channel_data_type = CL_UNORM_INT8; break;
        case 16: format.image_channel_data_type = CL_UNORM_INT16; break;
        default:
            cerr << "VolumeDescription(): unsupported bit depth "
                 << volume.bps() << endl;
            return;
    }

	size_t sdim[3] = { (size_t)dim[0], (size_t)dim[1], (size_t)dim[2] };
	image_ = new Image(sdim, &format, CL_MEM_READ_ONLY);
	image_->read(&data[0]);
#endif
}

// pass a four dimensional point, first three to account
// for rotation then final to account for translation.
double *VolumeDescription::localToGlobalCoordinateTrans(double *point) {
	double *transformedPoint = new double[4];
	transformedPoint[3] = 1; // set bottom entry to one
	for (int i = 0; i < 3; i++) transformedPoint[i] = point[i]; // copy the current vals

	// multiply by the scale correction matrix
	for (int i = 0; i < 4; i++){
		double cumulative = 0.0;
		for (int j  = 0; j < 4; j++){
			cumulative += (transformedPoint[j] * scaleCorrectionMatrix_[j][i]);
		}
		transformedPoint[i] = cumulative;
	}

	for (int y = 0; y < 4; y++) {
		double tempRowSum = 0.0;
		for (int x = 0; x < 4; x++) {
			tempRowSum += (minMaxLocalCorrection_[x][y] * transformedPoint[x]);
		}
		transformedPoint[y] = tempRowSum;
	}

	double tmp[4];

	// multiply rotation-translation matrix
	for (int y = 0; y < 4; y++) {
		double tempRowSum = 0.0;
		for (int x = 0; x < 4; x++) {
			tempRowSum += (globalToMinMaxLocal_[y][x] * transformedPoint[x]);
		}
		tmp[y] = tempRowSum;
	}

	tmp[2] = - tmp[2];
	tmp[1] = (height_ - tmp[1] - 1);
	tmp[0] = tmp[0];

	for (int y = 0; y < 3; y++)
		transformedPoint[y] = tmp[y] * scale_[y];

	return transformedPoint;
}

VolumeDescription::~VolumeDescription()
{
#ifdef WITH_CUDA
	cutilSafeCall(cudaFreeArray(image_));
#else
	if (image_) delete image_;
#endif
}

} } // namespace xromm::opencl

