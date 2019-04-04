/*
  This program detects image features using SIFT keypoints. For more info,
  refer to:
  
  Lowe, D. Distinctive image features from scale-invariant keypoints.
  International Journal of Computer Vision, 60, 2 (2004), pp.91--110.
  
  Copyright (C) 2006-2012  Rob Hess <rob@iqengines.com>

  Note: The SIFT algorithm is patented in the United States and cannot be
  used in commercial products without a license from the University of
  British Columbia.  For more information, refer to the file LICENSE.ubc
  that accompanied this distribution.

  Version: 1.1.2-20100521
*/

#include "sift.h"
#include "imgfeatures.h"
#include "utils.h"

#include <highgui.h>

//#include <unistd.h> //unix 标准头文件

#define OPTIONS ":o:m:i:s:c:r:n:b:dxh"

/*************************** Function Prototypes *****************************/

static void usage( char* );
static void arg_parse( int, char** );

/******************************** Globals ************************************/

char* pname;
char* img_file_name = "C:\\Users\\admin\\Pictures\\test1.jpg";
char* out_file_name = NULL;
char* out_img_name = NULL;
int intvls = SIFT_INTVLS;
double sigma = SIFT_SIGMA;
double contr_thr = SIFT_CONTR_THR;
int curv_thr = SIFT_CURV_THR;
int img_dbl = SIFT_IMG_DBL;
int descr_width = SIFT_DESCR_WIDTH;
int descr_hist_bins = SIFT_DESCR_HIST_BINS;
int display = 1;


/********************************** Main *************************************/

int main( int argc, char** argv )
{
  IplImage* img;
  struct feature* features;
  int n = 0;

  //arg_parse( argc, argv );

  fprintf( stderr, "Finding SIFT features...\n" );
  img = cvLoadImage( img_file_name, 1 );
  if( ! img )
    fatal_error( "unable to load image from %s", img_file_name );
  n = _sift_features( img, &features, intvls, sigma, contr_thr, curv_thr,
		      img_dbl, descr_width, descr_hist_bins );
  fprintf( stderr, "Found %d features.\n", n );
  
  if( display )
    {
      draw_features( img, features, n );
      display_big_img( img, img_file_name );
      cvWaitKey( 0 );
    }

  if( out_file_name != NULL )
    export_features( out_file_name, features, n );

  if( out_img_name != NULL )
    cvSaveImage( out_img_name, img, NULL );
  return 0;
}


/************************** Function Definitions *****************************/

// print usage for this program
static void usage( char* name )
{
  fprintf(stderr, "%s: detect SIFT keypoints in an image\n\n", name);
  fprintf(stderr, "Usage: %s [options] <img_file>\n", name);
  fprintf(stderr, "Options:\n");
  fprintf(stderr, "  -h               Display this message and exit\n");
  fprintf(stderr, "  -o <out_file>    Output keypoints to text file\n");
  fprintf(stderr, "  -m <out_img>     Output keypoint image file (format" \
	  " determined by extension)\n");
  fprintf(stderr, "  -i <intervals>   Set number of sampled intervals per" \
	  " octave in scale space\n");
  fprintf(stderr, "                   pyramid (default %d)\n",
	  SIFT_INTVLS);
  fprintf(stderr, "  -s <sigma>       Set sigma for initial gaussian"	\
	  " smoothing at each octave\n");
  fprintf(stderr, "                   (default %06.4f)\n", SIFT_SIGMA);
  fprintf(stderr, "  -c <thresh>      Set threshold on keypoint contrast" \
	  " |D(x)| based on [0,1]\n");
  fprintf(stderr, "                   pixel values (default %06.4f)\n",
	  SIFT_CONTR_THR);
  fprintf(stderr, "  -r <thresh>      Set threshold on keypoint ratio of" \
	  " principle curvatures\n");
  fprintf(stderr, "                   (default %d)\n", SIFT_CURV_THR);
  fprintf(stderr, "  -n <width>       Set width of descriptor histogram" \
	  " array (default %d)\n", SIFT_DESCR_WIDTH);
  fprintf(stderr, "  -b <bins>        Set number of bins per histogram" \
	  " in descriptor array\n");
  fprintf(stderr, "                   (default %d)\n", SIFT_DESCR_HIST_BINS);
  fprintf(stderr, "  -d               Toggle image doubling (default %s)\n",
	  SIFT_IMG_DBL == 0 ? "off" : "on");
  fprintf(stderr, "  -x               Turn off keypoint display\n");
}

