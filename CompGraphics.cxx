
/* This file contains the entirety of the software-based graphics system.
   It is divided in several sections, consisting of the different spaces and
   matrices required for it to work, as well as lighting variables, triangle
   objects to represent the individual triangles in the model, functions for
   shading and rasterizing, and building the model with a VTK file titled 
   "sample_geometry.vtk".

   It also creates 1000 pictures at different camera positions, to be placed in
   the directory where the program is run. With an application to build a video
   from the images, a short video can be made.
*/

// Max Aguirre
// May 2021

#include <iostream>
#include <algorithm>
#include <cmath>

#include <vtkDataSet.h>
#include <vtkDoubleArray.h>
#include <vtkImageData.h>
#include <vtkPNGWriter.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkPolyDataReader.h>
#include <vtkPoints.h>
#include <vtkUnsignedCharArray.h>
#include <vtkFloatArray.h>
#include <vtkCellArray.h>

#define NORMALS

using std::cerr;
using std::endl;

double ceil__round(double f)
{
    return ceil(f-0.00001);
}

double floor__round(double f)
{
    return floor(f+0.00001);
}


vtkImageData *
NewImage(int height, int width)
{
    vtkImageData *img = vtkImageData::New();
    img->SetDimensions(width, height, 1);
    img->AllocateScalars(VTK_UNSIGNED_CHAR, 3);

    return img;
}

void
WriteImage(vtkImageData *img, const char *filename)
{
   std::string full_filename = filename;
   full_filename += ".png";
   vtkPNGWriter *writer = vtkPNGWriter::New();
   writer->SetInputData(img);
   writer->SetFileName(full_filename.c_str());
   writer->Write();
   writer->Delete();
}


class Screen
{
  public:
    unsigned char   *buffer;
    double *depthBuffer;
    int width, height;

    // Screen Methods

    unsigned char getPixel(int pixel)
    {
      return buffer[pixel*3];
    }

    void setPixel(int pixel, int r, int g, int b)
    {

      int idx = 3*pixel;

      buffer[idx] = r;
      buffer[idx+1] = g;
      buffer[idx+2] = b;
    }

    double getDepth(int pixel)
    {
      return depthBuffer[pixel];
    }

    void setDepth(int pixel, double z)
    {
      depthBuffer[pixel] = z;
    }
};

class Matrix
{
  public:
    double          A[4][4];  // A[i][j] means row i, column j

    void TransformPoint(const double *ptIn, double *ptOut)
    {
      ptOut[0] = ptIn[0]*A[0][0] + ptIn[1]*A[1][0] + ptIn[2]*A[2][0] + ptIn[3]*A[3][0];
      ptOut[1] = ptIn[0]*A[0][1] + ptIn[1]*A[1][1] + ptIn[2]*A[2][1] + ptIn[3]*A[3][1];
      ptOut[2] = ptIn[0]*A[0][2] + ptIn[1]*A[1][2] + ptIn[2]*A[2][2] + ptIn[3]*A[3][2];
      ptOut[3] = ptIn[0]*A[0][3] + ptIn[1]*A[1][3] + ptIn[2]*A[2][3] + ptIn[3]*A[3][3];
    }
    
    static Matrix ComposeMatrices(const Matrix &M1, const Matrix &M2)
    {
      Matrix rv;
      for (int i = 0 ; i < 4 ; i++)
        for (int j = 0 ; j < 4 ; j++)
        {
          rv.A[i][j] = 0;
          for (int k = 0 ; k < 4 ; k++)
            rv.A[i][j] += M1.A[i][k]*M2.A[k][j];
        }

      return rv;
    }
    
    void Print(ostream &o)
    {
      for (int i = 0 ; i < 4 ; i++)
      {
        char str[256];
        sprintf(str, "(%.7f %.7f %.7f %.7f)\n", A[i][0], A[i][1], A[i][2], A[i][3]);
        o << str;
      }
    }
};


class Camera
{
  public:
    double          near, far;
    double          angle;
    double          position[3];
    double          focus[3];
    double          up[3];

    Matrix CameraTransform(void) 
    {
      // Create camera frame vectors
      double origin[3];
      double u[3];
      double v[3];
      double w[3];

      // Fill vectors Origin and W
      for(int i = 0; i < 3; i++)
      {
        origin[i] = position[i];
        w[i] = origin[i] - focus[i];
      }

      // Cross Up and W to get U
      u[0] = up[1]*w[2] - up[2]*w[1];
      u[1] = up[2]*w[0] - up[0]*w[2];
      u[2] = up[0]*w[1] - up[1]*w[0];

      // Cross w and u to get v
      v[0] = w[1]*u[2] - w[2]*u[1];
      v[1] = w[2]*u[0] - w[0]*u[2];
      v[2] = w[0]*u[1] - w[1]*u[0];
      
      // Normalize u, v, w
      double nU[3], nV[3], nW[3];

      for(int i = 0; i < 3; i++)
      {
        nU[i] = u[i]/sqrt(u[0]*u[0] + u[1]*u[1] + u[2]*u[2]);
        nV[i] = v[i]/sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
        nW[i] = w[i]/sqrt(w[0]*w[0] + w[1]*w[1] + w[2]*w[2]);
      }

      //cout << "U: " << nU[0] << " " << nU[1] << " " << nU[2] << endl;
      //cout << "V: " << nV[0] << " " << nV[1] << " " << nV[2] << endl; 
      //cout << "W: " << nW[0] << " " << nW[1] << " " << nW[2] << endl;
      //cout << "O: " << origin[0] << " " << origin[1] << " " << origin[2] << endl;

      //Make matrix to compose
      Matrix CTMatrix;

      //Initialize Matrix values to 0
      for (int i = 0 ; i < 4 ; i++)
        for (int j = 0 ; j < 4 ; j++)
          CTMatrix.A[i][j] = 0;

      // Define a t vector for the last coordinates
      double t[] = {0-origin[0], 0-origin[1], 0-origin[2]};

      // Create the matrix from everything we have so far
      for(int i = 0; i < 3; i++)
      {
        CTMatrix.A[i][0] = nU[i];
        CTMatrix.A[i][1] = nV[i];
        CTMatrix.A[i][2] = nW[i];
      }

      // Dot product each u,v,w and t
      CTMatrix.A[3][0] = nU[0]*t[0] + nU[1]*t[1] + nU[2]*t[2];
      CTMatrix.A[3][1] = nV[0]*t[0] + nV[1]*t[1] + nV[2]*t[2];
      CTMatrix.A[3][2] = nW[0]*t[0] + nW[1]*t[1] + nW[2]*t[2];
      CTMatrix.A[3][3] = 1;

      return CTMatrix;
    }

    Matrix ViewTransform(void) 
    {
      //Make matrix to compose
      Matrix VTMatrix;

      //Initialize Matrix values to 0
      for (int i = 0 ; i < 4 ; i++)
        for (int j = 0 ; j < 4 ; j++)
          VTMatrix.A[i][j] = 0;

      // Initialize special values
      VTMatrix.A[0][0] = 1/tan(angle/2);
      VTMatrix.A[1][1] = 1/tan(angle/2);
      VTMatrix.A[2][2] = (far+near)/(far-near);
      VTMatrix.A[2][3] = -1;
      VTMatrix.A[3][2] = (2*far*near)/(far-near);

      return VTMatrix;
    }

    Matrix DeviceTransform(Screen s) 
    {
      //Make matrix to compose
      Matrix DTMatrix;

      //Initialize Matrix values to 0
      for (int i = 0 ; i < 4 ; i++)
        for (int j = 0 ; j < 4 ; j++)
          DTMatrix.A[i][j] = 0;

      // Initialize special values
      DTMatrix.A[0][0] = s.width/2;
      DTMatrix.A[3][0] = s.width/2;
      DTMatrix.A[1][1] = s.height/2;
      DTMatrix.A[3][1] = s.height/2;
      DTMatrix.A[2][2] = 1;
      DTMatrix.A[3][3] = 1;

      return DTMatrix;
    }
};

double SineParameterize(int curFrame, int nFrames, int ramp)
{
    int nNonRamp = nFrames-2*ramp;
    double height = 1./(nNonRamp + 4*ramp/M_PI);
    if (curFrame < ramp)
    {
        double factor = 2*height*ramp/M_PI;
        double eval = cos(M_PI/2*((double)curFrame)/ramp);
        return (1.-eval)*factor;
    }
    else if (curFrame > nFrames-ramp)
    {
        int amount_left = nFrames-curFrame;
        double factor = 2*height*ramp/M_PI;
        double eval =cos(M_PI/2*((double)amount_left/ramp));
        return 1. - (1-eval)*factor;
    }
    double amount_in_quad = ((double)curFrame-ramp);
    double quad_part = amount_in_quad*height;
    double curve_part = height*(2*ramp)/M_PI;
    return quad_part+curve_part;
}

Camera GetCamera(int frame, int nframes)
{
    double t = SineParameterize(frame, nframes, nframes/10);
    Camera c;
    c.near = 5;
    c.far = 200;
    c.angle = M_PI/6;
    c.position[0] = 40*sin(2*M_PI*t);
    c.position[1] = 40*cos(2*M_PI*t);
    c.position[2] = 40;
    c.focus[0] = 0;
    c.focus[1] = 0;
    c.focus[2] = 0;
    c.up[0] = 0;
    c.up[1] = 1;
    c.up[2] = 0;
    return c;
}


struct LightingParameters
{
    LightingParameters(void)
    {
         lightDir[0] = -0.6;
         lightDir[1] = 0;
         lightDir[2] = -0.8;
         Ka = 0.3;
         Kd = 0.7;
         Ks = 2.8;
         alpha = 50.5;
    };
  

    double lightDir[3]; // The direction of the light source
    double Ka;          // The coefficient for ambient lighting
    double Kd;          // The coefficient for diffuse lighting
    double Ks;          // The coefficient for specular lighting
    double alpha;       // The exponent term for specular lighting
};

LightingParameters lp;

LightingParameters GetLighting(Camera c)
{
    LightingParameters lp;
    lp.lightDir[0] = c.position[0]-c.focus[0];
    lp.lightDir[1] = c.position[1]-c.focus[1];
    lp.lightDir[2] = c.position[2]-c.focus[2];
    double mag = sqrt(lp.lightDir[0]*lp.lightDir[0]
                    + lp.lightDir[1]*lp.lightDir[1]
                    + lp.lightDir[2]*lp.lightDir[2]);
    if (mag > 0)
    {
        lp.lightDir[0] /= mag;
        lp.lightDir[1] /= mag;
        lp.lightDir[2] /= mag;
    }

    return lp;
}


class Triangle
{
  public:
    double 		X[3];
    double 		Y[3];
    double 		Z[3];
    double 		colors[3][3];
    double    normals[3][3];
    double    shading[3];

    // Triangle Methods

    void PrintTrianglePoints()
    {
      cout << "Triangle points: " <<
      "(" << X[0] << ", " << Y[0] << ", " << Z[0] << ")," <<
      "(" << X[1] << ", " << Y[1] << ", " << Z[1] << ")," <<
      "(" << X[2] << ", " << Y[2] << ", " << Z[2] << ")\n";
    }

    void PrintTriangleColors()
    {
      cout << "Triangle colors: " << endl;
      for(int i = 0; i < 3; i++)
      {
        cout <<"(" << 
        255*colors[i][0] << ", " << 
        255*colors[i][1] << ", " << 
        255*colors[i][2] << ")" << endl;
      }
    }

    void GetMinMaxCols(double &min, double &max)
    {
      auto minMax = std::minmax({X[0], X[1], X[2]});

      min = ceil__round(minMax.first);
      max = floor__round(minMax.second);
    }

    void GetExactMinMaxCols(double &min, double &max)
    {
      auto minMax = std::minmax({X[0], X[1], X[2]});

      min = minMax.first;
      max = minMax.second;
    }

    void GetMinMaxRows(double &min, double &max)
    {
      auto minMax = std::minmax({X[0], X[1], X[2]});

      min = ceil__round(minMax.first);
      max = floor__round(minMax.second);
    }

    void GetExactMinMaxRows(double &min, double &max)
    {
      auto minMax = std::minmax({X[0], X[1], X[2]});

      min = minMax.first;
      max = minMax.second;
    }

    // This method implements the general Linear Interpolation formula
    // x is the coordinate of the point to LERP to, a is the coordinate from
    // the point to the left or above, b is the one to the right or below. 
    // fA and fB are the values at a, b, and fX is the calculated value at x
    void LERP(double x, double a, double b, double fA, double fB, double &fX)
    {
      fX = fA + ((x - a)/(b - a))*(fB - fA);
    }

    // This method returns the Y-Coords on the parameters
    // where the scanline at x intercepts the triangle slopes
    // for a "going right" triangle
    void GetGoingRightColIntercepts(double x, double &bot, double &top)
    {
      // y = mx + b, return y

      //Get ColMin and ColMax
      double colMin; 
      double colMax;
      GetExactMinMaxCols(colMin, colMax);

      // Check if points are on the left side of the triangle
      // If yes, denote them as top and bottom
      // Get slope from each to the third point

      std::vector<double> y_coords;

      double third_point_Y;

      for(int i = 0; i < 3; i++)
      {
        if(X[i] == colMin) {
          y_coords.push_back(Y[i]);
        }
        else {third_point_Y = Y[i];};
      }

      double bot_point = std::min(y_coords[0], y_coords[1]);
      double top_point = std::max(y_coords[0], y_coords[1]);

      double mbot = (third_point_Y - bot_point)/(colMax - colMin);
      double mtop = (third_point_Y - top_point)/(colMax - colMin);

      double bbot = third_point_Y - mbot*colMax;
      double btop = third_point_Y - mtop*colMax;

      bot = mbot*x + bbot;
      top = mtop*x + btop;
    }
    
    // This method returns the Y-Coords on the parameters
    // where the scanline at x intercepts the triangle slopes
    // for a "going left" triangle
    void GetGoingLeftColIntercepts(double x, double &bot, double &top)
    {
      // y = mx + b, return y

      //Get ColMin and ColMax
      double colMin; 
      double colMax;
      GetExactMinMaxCols(colMin, colMax);

      // Check if points are on the right side of the triangle
      // If yes, denote them as top and bottom
      // Get slope from each to the third point

      std::vector<double> y_coords;

      double third_point_Y;

      for(int i = 0; i < 3; i++)
      {
        if(X[i] == colMax) {
          y_coords.push_back(Y[i]);
        }
        else {third_point_Y = Y[i];};
      }

      double bot_point = std::min(y_coords[0], y_coords[1]);
      double top_point = std::max(y_coords[0], y_coords[1]);

      double mbot = (bot_point - third_point_Y)/(colMax - colMin);
      double mtop = (top_point - third_point_Y)/(colMax - colMin);

      double bbot = third_point_Y - mbot*colMin;
      double btop = third_point_Y - mtop*colMin;

      bot = mbot*x + bbot;
      top = mtop*x + btop;
    }

    // Find the Y-intercept at x of the opposite slope in an 
    // arbitrary triangle, where x is the x-coord of the middle
    // point and the slope is on this same coordinate
    void GetSplitColIntercepts(double x, double &slopeIntercept)
    {
      // y = mx + b, return y

      //Get ColMin and ColMax
      double colMin; 
      double colMax;
      GetExactMinMaxCols(colMin, colMax);

      // Define the points where the split will take place

      double third_point_Y = -1;
      double rise_Y_point = -1;

      for(int i = 0; i < 3; i++)
      {
        if(X[i] == colMin && rise_Y_point == -1) {
          //y_coords.push_back(Y[i]);
          rise_Y_point = Y[i];
        }
        else if(X[i] == colMax && third_point_Y == -1)
        {
          //y_coords.push_back(Y[i]);
          third_point_Y = Y[i];
        }
      }

      double m = (third_point_Y - rise_Y_point)/(colMax - colMin);

      double b = third_point_Y - m*colMax;

      slopeIntercept = m*x + b;
    }

    // Rasterize going right triangles
    void RasterizeGoingRight(Screen screen, int npixels)
    {
      // Get ColMin and ColMax
      double colMin; 
      double colMax;
      GetMinMaxCols(colMin, colMax);

      // Scanline Algorithm
      for(int c = colMin; c <= std::min((int)colMax, screen.width-1); c++)
      {

        // Get Slope of triangle
        double bot;
        double top;

        GetGoingRightColIntercepts((double)c, bot, top);

        double botEnd = ceil__round(bot);
        double topEnd = floor__round(top);

        //Interpolate Z values for bottom, top ends
        double botEndZ;
        double topEndZ;

        LERP(double(c), X[0], X[1], Z[0], Z[1], botEndZ);
        LERP(double(c), X[0], X[2], Z[0], Z[2], topEndZ);

        //cout << "Y values at scanline " << c << ": Bot:" << bot << ", Top: " << top << endl;
        //cout << "Z values at scanline " << c << ": Bot:" << botEndZ << ", Top: " << topEndZ << endl;

        //Interpolate Shading values for bottom, top ends
        double botEndShading;
        double topEndShading;

        LERP(double(c), X[0], X[1], shading[0], shading[1], botEndShading);
        LERP(double(c), X[0], X[2], shading[0], shading[2], topEndShading);

        //Interpolate colors for bottom, top ends
        double botEndColor[3];
        double topEndColor[3];

        for(int i = 0; i < 3; i++)
        {
          LERP(double(c), X[1], X[0], colors[1][i], colors[0][i], 
               topEndColor[i]);

          LERP(double(c), X[2], X[0], colors[2][i], colors[0][i], 
               botEndColor[i]);
        }

        /*cout << "Colors at scanline: " << c << endl;
        cout << "Bot color:" << "[" << botEndColor[0] << ", "
        << botEndColor[1] << ", " << botEndColor[2] << "]" << endl;
        cout << "Top color:" << "[" << topEndColor[0] << ", "
        << topEndColor[1] << ", " << topEndColor[2] << "]" << endl;*/


        for(int r = botEnd; r <= std::min((int)topEnd, screen.height-1); r++)
        { 
          // Interpolate Z value from bottom, top ends
          double pointZ;
          LERP(double(r), bot, top, botEndZ, topEndZ, pointZ);

          // Interpolate shading value from bottom, top ends
          double pointShading;
          LERP(double(r), bot, top, botEndShading, topEndShading, pointShading);

          // Interpolate pixel color from bottom, top ends
          double pointColors[3];

          for(int i = 0; i < 3; i++)
          {
            LERP(double(r), bot, top, botEndColor[i], topEndColor[i], 
                 pointColors[i]);

            pointColors[i] = ceil__round(std::min(1.0, pointColors[i]*pointShading)*255);
          }

          /*cout << "Colors at point (" << c << ", " << r << ")" << endl;
          cout << "[" << pointColors[0] << ", "
          << pointColors[1] << ", " << pointColors[2] << "]" << endl;*/


          // Multiply r by screen witdh for it to represent
          // The pixels at that row
          int cur_pix = c+r*screen.width;

          // Check if new color is closer than new color. If so,
          // Color over it.
          if(pointZ > screen.getDepth(cur_pix))
          {
            // Assign new depth
            screen.setDepth(cur_pix, pointZ);

            if((c >= 0 && c < screen.width) && (r >= 0 && r < screen.height))
            {
              screen.setPixel(cur_pix, pointColors[0],
                                       pointColors[1],
                                       pointColors[2]);
            }
          }
        }
      }
    }
    
    // Rasterize going left triangles
    void RasterizeGoingLeft(Screen screen, int npixels)
    {
      // Get ColMin and ColMax
      double colMin; 
      double colMax;
      GetMinMaxCols(colMin, colMax);

      // Scanline Algorithm
      for(int c = colMin; c <= std::min((int)colMax, screen.width-1); c++)
      {

        // Get Slope of triangle
        double bot;
        double top;

        //cout << "Left Scanline at: " << c << endl;
        GetGoingLeftColIntercepts((double)c, bot, top);

        double botEnd = ceil__round(bot);
        double topEnd = floor__round(top);

        //Interpolate Z values for bottom, top ends
        double botEndZ;
        double topEndZ;

        LERP(double(c), X[0], X[1], Z[0], Z[1], botEndZ);
        LERP(double(c), X[0], X[2], Z[0], Z[2], topEndZ);

        /*cout << "\nScanline at " << c << ": ";
        cout << "Y Bot:" << bot << ", Top: " << top << "; ";
        cout << "Z Bot:" << botEndZ << ", Top: " << topEndZ << endl;*/

        //Interpolate Shading values for bottom, top ends
        double botEndShading;
        double topEndShading;

        LERP(double(c), X[0], X[1], shading[0], shading[1], botEndShading);
        LERP(double(c), X[0], X[2], shading[0], shading[2], topEndShading);

        //Interpolate colors for bottom, top ends
        double botEndColor[3];
        double topEndColor[3];

        for(int i = 0; i < 3; i++)
        {
          LERP(double(c), X[0], X[1], colors[0][i], colors[1][i], 
               topEndColor[i]);

          LERP(double(c), X[0], X[2], colors[0][i], colors[2][i], 
               botEndColor[i]);
        }

        //cout << "Colors at scanline: " << c << endl;
        /*cout << "Bot color:" << "[" << botEndColor[0] << ", "
        << botEndColor[1] << ", " << botEndColor[2] << "]" << ", ";
        cout << "Top color:" << "[" << topEndColor[0] << ", "
        << topEndColor[1] << ", " << topEndColor[2] << "]" << endl;*/

        for(int r = botEnd; r <= std::min((int)topEnd, screen.height-1); r++)
        { 
          // Interpolate Z value from bottom, top ends
          double pointZ;
          LERP(double(r), bot, top, botEndZ, topEndZ, pointZ);

          // Interpolade Shading from bottom, top ends
          double pointShading;
          LERP(double(r), bot, top, botEndShading, topEndShading, pointShading);

          // Interpolate pixel color from bottom, top ends
          double pointColors[3];

          for(int i = 0; i < 3; i++)
          {
            LERP(double(r), bot, top, botEndColor[i], topEndColor[i], 
                 pointColors[i]);

            pointColors[i] = ceil__round(std::min(1.0, pointColors[i]*pointShading)*255);
          }

          /*cout << "Colors at point (" << c << ", " << r << ")" << endl;
          cout << "[" << pointColors[0] << ", "
          << pointColors[1] << ", " << pointColors[2] << "]" << endl;*/

          // Multiply r by width for it to represent
          // The pixels at that row, get current pixel
          int cur_pix = c+r*screen.width;

          // Check if new color is closer than new color. If so,
          // Color over it.
          if(pointZ > screen.getDepth(cur_pix))
          {
            // Assign new depth
            screen.setDepth(cur_pix, pointZ);

            // Assign color to pixel.
            if((c >= 0 && c < screen.width) && (r >= 0 && r < screen.height))
            {
              screen.setPixel(cur_pix, pointColors[0],
                                       pointColors[1],
                                       pointColors[2]);
            }
          }
        }
      }
    }


    // Split the triangle into one going left
    // and one going right, then rasterize them
    void Rasterize(Screen screen, int npixels)
    {
      // Get ColMin and ColMax
      double exColMin; 
      double exColMax;
      GetExactMinMaxCols(exColMin, exColMax);

      // Find which index contains what point
      // Initialized to -1 in case two lie on the same column
      int colMinPointIdx = -1;
      int midPointIdx = -1;
      int colMaxPointIdx = -1;

      for(int i = 0; i < 3; i++)
      {
        if(X[i] == exColMin && colMinPointIdx == -1)
        {
          colMinPointIdx = i;
        }
        else if (X[i] == exColMax && colMaxPointIdx == -1)
        {
          colMaxPointIdx = i;
        }
        else
        {
          midPointIdx = i;
        }
      }

      // Find the Y_intercept in the slope of the triangle at the middle point
      double midPointTopY;
      double midPointBotY;

      double slopeIntercept;
      GetSplitColIntercepts(X[midPointIdx], slopeIntercept);

      midPointBotY = std::min(slopeIntercept, Y[midPointIdx]);
      midPointTopY = std::max(slopeIntercept, Y[midPointIdx]);

      // Find the values in the Z coordinates to pass on to new triangles
      double midPointTopZ;
      double midPointBotZ;

      if(Y[midPointIdx] == midPointTopY)
      {
        // If the middle vertex is at the top, interpolate for bottom
        LERP(X[midPointIdx], X[colMinPointIdx], X[colMaxPointIdx], Z[colMinPointIdx], Z[colMaxPointIdx], midPointBotZ);
        midPointTopZ = Z[midPointIdx];
      }
      else
      {
        // The middle vertex is at the bottom, interpolate for top
        LERP(X[midPointIdx], X[colMinPointIdx], X[colMaxPointIdx], Z[colMinPointIdx], Z[colMaxPointIdx], midPointTopZ);
        midPointBotZ = Z[midPointIdx];
      }

      // Interpolate colors based on the top and bottom scanline points
      double midPointTopColor[3];
      double midPointBotColor[3];

      if(Y[midPointIdx] == midPointTopY)
      {    
        // If middle vertex is at top, interpolate for bottom
        for(int i = 0; i < 3; i++)
        {
           LERP(X[midPointIdx], X[colMinPointIdx], X[colMaxPointIdx], 
                colors[colMinPointIdx][i], colors[colMaxPointIdx][i], 
                midPointBotColor[i]);

           midPointTopColor[i] = colors[midPointIdx][i];
        }
      }
      else
      {
        // If middle vertex is at bottom, interpolate for top
        for(int i = 0; i < 3; i++)
        {
           LERP(X[midPointIdx], X[colMinPointIdx], X[colMaxPointIdx], 
                colors[colMinPointIdx][i], colors[colMaxPointIdx][i], 
                midPointTopColor[i]);

           midPointBotColor[i] = colors[midPointIdx][i];
        }
      }


      //cout << "X value at Left: " << X[colMinPointIdx] << ", Right: " << X[colMaxPointIdx] << endl;
      //cout << "Y value at Top: " << midPointTopY << ", bottom: " << midPointBotY << endl;
      //cout << "Z value at top: " << midPointTopZ << ", Bottom: " << midPointBotZ << endl << endl;

      // Make new triangles and set their X Y Z coordinates
      // For these new triangles:
      //  Index 0 is the point that is not along the split-line
      //  Index 1 is the point at the bottom of the split-line
      //  Index 2 is the point at the top of the split-line


      // Interpolate Shading values
      double midPointTopShading;
      double midPointBotShading;

      if(Y[midPointIdx] == midPointTopY)
      {
        // If the middle vertex is at the top, interpolate for bottom
        LERP(X[midPointIdx], X[colMinPointIdx], X[colMaxPointIdx], shading[colMinPointIdx], shading[colMaxPointIdx], midPointBotShading);
        midPointTopShading = shading[midPointIdx];
      }
      else
      {
        // The middle vertex is at the bottom, interpolate for top
        LERP(X[midPointIdx], X[colMinPointIdx], X[colMaxPointIdx], shading[colMinPointIdx], shading[colMaxPointIdx], midPointTopShading);
        midPointBotShading = shading[midPointIdx];
      }

      //cout << "Shading values: " << endl;
      //cout << "MidPoint top:" << midPointTopShading << " MidPoint bot:" << midPointBotShading << endl;


      Triangle left = Triangle();  // Triangle to the left of split
      Triangle right = Triangle(); // Triangle to the right of split

      left.X[0] = X[colMinPointIdx];
      left.X[1] = X[midPointIdx];
      left.X[2] = X[midPointIdx];

      left.Y[0] = Y[colMinPointIdx];
      left.Y[1] = midPointBotY;
      left.Y[2] = midPointTopY;

      left.Z[0] = Z[colMinPointIdx];
      left.Z[1] = midPointBotZ;
      left.Z[2] = midPointTopZ;

      left.shading[0] = shading[colMinPointIdx];
      left.shading[1] = midPointBotShading;
      left.shading[2] = midPointTopShading;

      right.X[0] = X[colMaxPointIdx];
      right.X[1] = X[midPointIdx];
      right.X[2] = X[midPointIdx];

      right.Y[0] = Y[colMaxPointIdx];
      right.Y[1] = midPointBotY;
      right.Y[2] = midPointTopY;

      right.Z[0] = Z[colMaxPointIdx];
      right.Z[1] = midPointBotZ;
      right.Z[2] = midPointTopZ;

      right.shading[0] = shading[colMaxPointIdx];
      right.shading[1] = midPointBotShading;
      right.shading[2] = midPointTopShading;

      left.colors[0][0] = colors[colMinPointIdx][0];
      left.colors[0][1] = colors[colMinPointIdx][1];
      left.colors[0][2] = colors[colMinPointIdx][2];
      left.colors[1][0] = midPointTopColor[0];
      left.colors[1][1] = midPointTopColor[1];
      left.colors[1][2] = midPointTopColor[2];
      left.colors[2][0] = midPointBotColor[0];
      left.colors[2][1] = midPointBotColor[1];
      left.colors[2][2] = midPointBotColor[2];

      right.colors[0][0] = colors[colMaxPointIdx][0];
      right.colors[0][1] = colors[colMaxPointIdx][1];
      right.colors[0][2] = colors[colMaxPointIdx][2];
      right.colors[1][0] = midPointTopColor[0];
      right.colors[1][1] = midPointTopColor[1];
      right.colors[1][2] = midPointTopColor[2];
      right.colors[2][0] = midPointBotColor[0];
      right.colors[2][1] = midPointBotColor[1];
      right.colors[2][2] = midPointBotColor[2];

      /*cout << "Left Triangle:" << endl;
      left.PrintTrianglePoints();
      cout << "Right Triangle:" << endl;
      right.PrintTrianglePoints();
      cout << endl;*/

      /*cout << "Left Triangle:" << endl;
      left.PrintTriangleColors();
      cout << "Right Triangle:" << endl;
      right.PrintTriangleColors();
      cout << endl;*/

      // Rasterize the new triangles

      left.RasterizeGoingLeft(screen, npixels);
      right.RasterizeGoingRight(screen, npixels);

    }

    // Calculate the shading value of vertex
    double CalculateShading(LightingParameters &lp,
                          double *viewDirection, double *normal)
    {
      double shadingAmount; //Ka + Kd*Diffuse + Ks*Specular
      double diffuse;   // Kd * Dot product L N = cos(a) (Normalize L, N)
      double specular;  // Ks * cos(a)^alpha

    /* ======================== Diffuse Calculation ======================== */

      double normLightDir[3];
      double normNormal[3];

      // Normalize the normal and lightDir vectors
      for(int i = 0; i < 3; i++)
      {
        normLightDir[i] = 
          lp.lightDir[i]/sqrt(lp.lightDir[0]*lp.lightDir[0] + 
                              lp.lightDir[1]*lp.lightDir[1] + 
                              lp.lightDir[2]*lp.lightDir[2]);

        normNormal[i] = 
          normal[i]/sqrt(normal[0]*normal[0] +  
                         normal[1]*normal[1] + 
                         normal[2]*normal[2]);
      }

      // Dot Product vertex normal and lightDir
      diffuse = normLightDir[0]*normNormal[0] +
                normLightDir[1]*normNormal[1] +
                normLightDir[2]*normNormal[2];

      if(diffuse < 0)
        diffuse = 0;

    /* ======================= Specular Calculation ======================== */

      double lightR[3];
      double normLightR[3];
      double normPointV[3];

      // L.N are calculated in diffuse! So normalize V then put it all together

      //cout << "First time?" << endl;

      for(int i = 0; i < 3; i++)
      {
        normPointV[i] = 
          viewDirection[i]/sqrt(viewDirection[0]*viewDirection[0] + 
                                viewDirection[1]*viewDirection[1] + 
                                viewDirection[2]*viewDirection[2]);
      }

      // Calculate R
      for(int i = 0; i < 3; i++)
      {
        lightR[i] = 2*(diffuse)*normal[i] - lp.lightDir[i];
      }

      // Normalize R
      for(int i = 0; i < 3; i++)
      {
        normLightR[i] = 
          lightR[i]/sqrt(lightR[0]*lightR[0] + 
                         lightR[1]*lightR[1] + 
                         lightR[2]*lightR[2]);
      }

      // Dot product V, R
      specular = normPointV[0]*normLightR[0] +
                 normPointV[1]*normLightR[1] +
                 normPointV[2]*normLightR[2];
      
      // Shinyness coefficient
      if(specular < 0)
        specular = 0;

      specular = pow(specular, lp.alpha);

    /* ========================= Final Calculation ========================== */
      shadingAmount = lp.Ka + lp.Kd*diffuse + lp.Ks*specular;

      return shadingAmount;
    }
};

// Triangle routine
std::vector<Triangle>
GetTriangles(void)
{
    vtkPolyDataReader *rdr = vtkPolyDataReader::New();
    rdr->SetFileName("sample_geometry.vtk");
    cerr << "Reading" << endl;
    rdr->Update();
    cerr << "Done reading" << endl;
    if (rdr->GetOutput()->GetNumberOfCells() == 0)
    {
        cerr << "Unable to open file!!" << endl;
        exit(EXIT_FAILURE);
    }
    vtkPolyData *pd = rdr->GetOutput();

    int numTris = pd->GetNumberOfCells();
    vtkPoints *pts = pd->GetPoints();
    vtkCellArray *cells = pd->GetPolys();
    vtkDoubleArray *var = (vtkDoubleArray *) pd->GetPointData()->GetArray("hardyglobal");
    double *color_ptr = var->GetPointer(0);
    //vtkFloatArray *var = (vtkFloatArray *) pd->GetPointData()->GetArray("hardyglobal");
    //float *color_ptr = var->GetPointer(0);
    vtkFloatArray *n = (vtkFloatArray *) pd->GetPointData()->GetNormals();
    float *normals = n->GetPointer(0);
    std::vector<Triangle> tris(numTris);
    vtkIdType npts;
    vtkIdType *ptIds;
    int idx;
    for (idx = 0, cells->InitTraversal() ; cells->GetNextCell(npts, ptIds) ; idx++)
    {
        if (npts != 3)
        {
            cerr << "Non-triangles!! ???" << endl;
            exit(EXIT_FAILURE);
        }
        double *pt = NULL;
        pt = pts->GetPoint(ptIds[0]);
        tris[idx].X[0] = pt[0];
        tris[idx].Y[0] = pt[1];
        tris[idx].Z[0] = pt[2];
        #ifdef NORMALS
        tris[idx].normals[0][0] = normals[3*ptIds[0]+0];
        tris[idx].normals[0][1] = normals[3*ptIds[0]+1];
        tris[idx].normals[0][2] = normals[3*ptIds[0]+2];
        #endif
        pt = pts->GetPoint(ptIds[1]);
        tris[idx].X[1] = pt[0];
        tris[idx].Y[1] = pt[1];
        tris[idx].Z[1] = pt[2];
        #ifdef NORMALS
        tris[idx].normals[1][0] = normals[3*ptIds[1]+0];
        tris[idx].normals[1][1] = normals[3*ptIds[1]+1];
        tris[idx].normals[1][2] = normals[3*ptIds[1]+2];
        #endif
        pt = pts->GetPoint(ptIds[2]);
        tris[idx].X[2] = pt[0];
        tris[idx].Y[2] = pt[1];
        tris[idx].Z[2] = pt[2];
        #ifdef NORMALS
        tris[idx].normals[2][0] = normals[3*ptIds[2]+0];
        tris[idx].normals[2][1] = normals[3*ptIds[2]+1];
        tris[idx].normals[2][2] = normals[3*ptIds[2]+2];
        #endif

        // 1->2 interpolate between light blue, dark blue
        // 2->2.5 interpolate between dark blue, cyan
        // 2.5->3 interpolate between cyan, green
        // 3->3.5 interpolate between green, yellow
        // 3.5->4 interpolate between yellow, orange
        // 4->5 interpolate between orange, brick
        // 5->6 interpolate between brick, salmon
        double mins[7] = { 1, 2, 2.5, 3, 3.5, 4, 5 };
        double maxs[7] = { 2, 2.5, 3, 3.5, 4, 5, 6 };
        unsigned char RGB[8][3] = { { 71, 71, 219 }, 
                                    { 0, 0, 91 },
                                    { 0, 255, 255 },
                                    { 0, 128, 0 },
                                    { 255, 255, 0 },
                                    { 255, 96, 0 },
                                    { 107, 0, 0 },
                                    { 224, 76, 76 } 
                                  };
        for (int j = 0 ; j < 3 ; j++)
        {
            float val = color_ptr[ptIds[j]];
            int r;
            for (r = 0 ; r < 7 ; r++)
            {
                if (mins[r] <= val && val < maxs[r])
                    break;
            }
            if (r == 7)
            {
                cerr << "Could not interpolate color for " << val << endl;
                exit(EXIT_FAILURE);
            }
            double proportion = (val-mins[r]) / (maxs[r]-mins[r]);
            tris[idx].colors[j][0] = (RGB[r][0]+proportion*(RGB[r+1][0]-RGB[r][0]))/255.0;
            tris[idx].colors[j][1] = (RGB[r][1]+proportion*(RGB[r+1][1]-RGB[r][1]))/255.0;
            tris[idx].colors[j][2] = (RGB[r][2]+proportion*(RGB[r+1][2]-RGB[r][2]))/255.0;
        }
    }

    return tris;
}

int main()
{
  // Initialize VTK Image
  vtkImageData *image = NewImage(1000, 1000);
  unsigned char *buffer = 
   (unsigned char *) image->GetScalarPointer(0,0,0);
  int npixels = 1000*1000;
  double depthBuffer[npixels];

  // Create Triangles
  std::vector<Triangle> triangles = GetTriangles();

  // Create a Screen object
  Screen screen;
  screen.width = 1000;
  screen.height = 1000;
  
  // Create camera positions
  for(int i = 0; i < 1000; i++)
  {

    // Reset buffers
    for (int j = 0 ; j < npixels*3 ; j++)
      buffer[j] = 0;
    
    for(int j = 0; j < npixels; j++)
      depthBuffer[j] = -1;

    // Reinitialize screen buffers
    screen.buffer = buffer;
    screen.depthBuffer = depthBuffer;

    Camera c = GetCamera(i, 1000);

    // Calculate the matrices to be composed
    Matrix worldToCam = c.CameraTransform();
    Matrix camToImage = c.ViewTransform();
    Matrix imgToDevice = c.DeviceTransform(screen);

    // Compose matrices into M
    Matrix M = Matrix::ComposeMatrices(Matrix::ComposeMatrices(worldToCam, camToImage), imgToDevice);

    // Loop through each triangle and rasterize it
    for(int i = 0; i < triangles.size(); i++)
    {

      double vertex0[] = {triangles[i].X[0], triangles[i].Y[0], triangles[i].Z[0], 1};
      double vertex1[] = {triangles[i].X[1], triangles[i].Y[1], triangles[i].Z[1], 1};
      double vertex2[] = {triangles[i].X[2], triangles[i].Y[2], triangles[i].Z[2], 1};

      LightingParameters lightParam = GetLighting(c);
      double viewDirection[3][3];

      // Loop each coordinate. Vertex assigned manually.
      for(int j = 0; j < 3; j++)
      {
        viewDirection[0][j] = c.position[j] - vertex0[j];
        viewDirection[1][j] = c.position[j] - vertex1[j];
        viewDirection[2][j] = c.position[j] - vertex2[j];
      }

      // Calculate shading for each vertex
      for(int j = 0; j < 3; j++)
      {
        triangles[i].shading[j] = 
          triangles[i].CalculateShading(lightParam, viewDirection[j],
                                         triangles[i].normals[j]);
      }

      double newVertex0[4];
      double newVertex1[4];
      double newVertex2[4];

      //Apply Matrix to triangle vertices;
      M.TransformPoint(vertex0, newVertex0);
      M.TransformPoint(vertex1, newVertex1);
      M.TransformPoint(vertex2, newVertex2);

      // Copy values before assignation
      double xCopy[3];
      double yCopy[3];
      double zCopy[3];

      for(int j = 0; j < 3; j++)
      {
        xCopy[j] = triangles[i].X[j];
        yCopy[j] = triangles[i].Y[j];
        zCopy[j] = triangles[i].Z[j];
      }

      // Assign the new coordinates to the triangles again
      triangles[i].X[0] = newVertex0[0]/newVertex0[3];
      triangles[i].X[1] = newVertex1[0]/newVertex1[3];
      triangles[i].X[2] = newVertex2[0]/newVertex2[3];
      triangles[i].Y[0] = newVertex0[1]/newVertex0[3];
      triangles[i].Y[1] = newVertex1[1]/newVertex1[3];
      triangles[i].Y[2] = newVertex2[1]/newVertex2[3];
      triangles[i].Z[0] = newVertex0[2]/newVertex0[3];
      triangles[i].Z[1] = newVertex1[2]/newVertex1[3];
      triangles[i].Z[2] = newVertex2[2]/newVertex2[3];

      // Rasterize Triangle
      triangles[i].Rasterize(screen, npixels);

      //Restore original triangle values
      for(int j = 0; j < 3; j++)
      {
        triangles[i].X[j] = xCopy[j];
        triangles[i].Y[j] = yCopy[j];
        triangles[i].Z[j] = zCopy[j];
      }
    }

    char buffer[50];
    int success;

    success = sprintf(buffer, "test%03d", i);

    if(success < 0)
    {
      printf("Error: Frame %s could not be printed\n", buffer);
    }

    WriteImage(image, buffer);
  }
}
