///
/// CS-583: Introduction to Computer Vision
/// https://www.cs.drexel.edu/~kon/introcompvis/
///
/// Project 2 - Panorama 
///

// Needed for reading files
import java.io.FileNotFoundException;
import java.util.*;
// Needed for the various Matrix and Vector operations
import no.uib.cipr.matrix.*;
// Needed for functions like Max and Min
import java.lang.Math.*;
// Needed for the .ini file parsing
import org.ini4j.*;

//
// Constants
//

// ini file reading constants including filename and key names
static final String DEFAULT_INI_FILE = "demo.ini";
static final String PARAMETERS = "Camera Parameters";
static final String FOCAL_LENGTH = "focal_length";
static final String RADIAL_DISTORTION = "radial_distortion";
static final String IMAGES = "Images";
static final String OUTPUT_IMAGE = "result";
static final String IMAGE = "image";
static final String PANORAMA = "Panorama";
static final String PAIR = "pair";
static final String LUCAS_KANADE = "Lucas-Kanade";
static final String LEVELS = "levels";
static final String STEPS = "steps";

// Prefix to append to each warped image
static final String WARPED = "WARPED_";

// (Suggestion) ini file that Lucas-Kanade portion generates
static final String LK_POINTS = "Lucas-Kanade-Output.ini";

// Final image name
static final String RESULT = "panorama.png";

// (Suggestion) Toggle sections of code to help with debugging
static final boolean DO_WARP = true;
static final boolean DO_LK = true;
static final boolean DO_PANORAMA = true;

// Homographic indexes
static final int X = 0;
static final int Y = 1;
static final int Z = 2;

// Whether or not to print debug information
static final boolean DEBUG = true;

// Image to show
PImage showImage;
double slope;

//
// Functions
//

///
/// Set up screen size, turn off re-drawing loop
///
void setup() {
  size(900, 900);
  noLoop();
  
  // Read .ini file
  Ini ini = new Ini();
  try {
    BufferedReader reader = createReader(DEFAULT_INI_FILE);
    if(reader == null) {
      throw new FileNotFoundException(DEFAULT_INI_FILE);
    }
    ini.load(reader);

    //
    // Warp images all images in IMAGES section using parameters in PARAMETERS section.
    // (call warpCylindrical for each image)
    //
    if(DO_WARP) {
      Ini.Section images = ini.get(IMAGES);
      java.util.List imageNames = images.getAll(IMAGE);

      // Read parameters
      Ini.Section parameters = ini.get(PARAMETERS);

      int focalLength = int(parameters.get(FOCAL_LENGTH));
      // Get all distortion parameters as an array of floats
      float[] distortionParameters = float(parameters.getAll(RADIAL_DISTORTION).toArray(new String[0]));
      for(java.util.Iterator iterator = imageNames.iterator(); iterator.hasNext();) {
        String filename = (String)iterator.next();
        PImage image = loadImage(filename);
        PImage corrected = warpCylindrical(image, focalLength, distortionParameters);
        
        String newName = savePath("WARPED_"+filename);
     //   String newName = WARPED + filename;
        corrected.save(newName);
        if(DEBUG){
          println("Saved '" + newName + "'");
        }
        showImage = corrected; // debug
      }
    } else {
      println("Skipping warping step.");
    }


    //
    // Run Lucas-Canade on all PAIRs in the PANORAMA section
    // (Call lucasKanade for each image pair, and save the results to a file)
    //
    if(DO_LK) {
      Ini.Section images = ini.get(IMAGES);
      java.util.List imageNames = images.getAll(IMAGE);
      java.util.List pairs = ini.get(PANORAMA).getAll(PAIR);
      Ini.Section settings = ini.get(LUCAS_KANADE);
      int levels = int(settings.get(LEVELS));
      int steps = int(settings.get(STEPS));

      // Create an ini file to write LK results
      PrintWriter pointWriter = createWriter(LK_POINTS);
      pointWriter.println("[" + PANORAMA + "]");

      for(java.util.Iterator iterator = pairs.iterator(); iterator.hasNext();) {
        float[] values = float(splitTokens((String)iterator.next(), " "));

        DenseVector translation = new DenseVector( new double[]{values[2], values[3]});
        lucasKanade(loadImage(WARPED + images.get(IMAGE, (int)values[0])),
                    loadImage(WARPED + images.get(IMAGE, (int)values[1])),
                    levels, steps, translation);

        PImage translated = translate(loadImage(WARPED + images.get(IMAGE, (int)values[1])), translation);
        translated.save("translated.png");
        loadImage(WARPED + images.get(IMAGE, (int)values[0])).save("target.png");
        pointWriter.println(PAIR + " = " + (int)values[0] + " " + (int)values[1] + " " +
                            translation.get(X) + " " + translation.get(Y));
      }
      pointWriter.close();
    } else {
      println("Skipping Lucas-Kanade step.");
    }

    //
    // Stitch image together
    // (Read the Lucas-Kanade generated file into an array of images, and a matrix of translations
    //  then call makePanorama)
    //
    if(DO_PANORAMA) {
      // read the LK generated ini file
      Ini LKini = new Ini();
      BufferedReader LKreader = createReader(LK_POINTS);
      if(LKreader == null) {
          throw new FileNotFoundException(LK_POINTS);
      }
      LKini.load(LKreader);

      Ini.Section images = ini.get(IMAGES);
      Ini.Section pairs = LKini.get(PANORAMA);
      int pairCount = (pairs.getAll(PAIR)).size();
      PImage[] imageArray = new PImage[pairCount];
      
      DenseMatrix translations = new DenseMatrix(pairCount, 2);
      int previousImageIndex = -1;
      for(int pair = 0; pair < pairCount; pair ++) {
        float[] values = float(splitTokens((String)pairs.get(PAIR, pair), " "));
        if(pair > 0 && (int)values[0] != previousImageIndex){
          throw new RuntimeException("You must specify your pairs in order.");
        }
        int imageIndex = (int)values[1];
        imageArray[pair] = loadImage(WARPED + images.get(IMAGE, (int)values[0]));
        translations.set(pair, X, values[2]);
        translations.set(pair, Y, values[3]);
        previousImageIndex = imageIndex;
      }
      
           
      PImage panorama = makePanorama(imageArray, translations);
            showImage = panorama;

      String filename = images.get(OUTPUT_IMAGE);
      filename = filename == null ? RESULT : filename;
      panorama.save(filename);
      println("Saved result as '" + filename + "'");
    } else {
      println("Skipping panorama creation step.");
    }
  } catch(FileNotFoundException e) {
    println("You must have the file '" + e.getMessage() + "' present.");
  } catch(IOException e) {
    System.err.println("Failure reading ini file: " + e.getMessage());
    exit();
  }
  println("Done.");
}


///
/// Main function
///
void draw() {
  background(0);

  float divideBy = 1.0;
  if(showImage.width > width || showImage.height > height) {
      divideBy = max(float(showImage.width)/width, float(showImage.height)/height);
  }

  int originX = floor((width-showImage.width/divideBy)/2);
  int originY = floor((height-showImage.height/divideBy)/2);

  // Show the picture scaled appropriately
  image(showImage, originX, originY, showImage.width/divideBy, showImage.height/divideBy);
}


///
/// Warps the given 'image' to cylindrical coordinates using the camera parameter
/// @param image Image to warp
/// @param focalLength Focal distance (in pixels)
/// @param distortionParameters Ordered radial distortion parameters
/// @returns 'image' has been warped to cylindrical coordinates and has had radial distortion removed
///
PImage warpCylindrical(PImage image, float focalLength, float[] radialCoefficients) {
  PImage result = createImage(image.width, image.height, RGB);
  float xc = image.width / 2;
  float yc = image.height / 2;
  //
  // Loop for all cylindrical coordinates
  //
for(int yi = 0; yi < image.height; yi++){
    for(int xi = 0; xi < image.width; xi++){
      //
      // Convert to cylindrical co-ordinates (\theta,h)
      //
      float theta = (xi - xc) / focalLength;
      float h = (yi - yc) / focalLength;
      
      //
      // Convert to 3d cylindrical points (\hat{x}, \hat{y})
      //
      float xhat = sin(theta);
      float yhat = h;
      float zhat = cos(theta);
      
      //
      // Inverse map back to (x,y) points
      //
      float x = xhat / zhat;
      float y = yhat / zhat;

     
  
  // Account for radial distortion (you can assume there are exactly 2 coefficients)
  float x_prime;
      float y_prime;
      float r2 = x * x + y * y;
      float r4 = r2 * r2;
      
      x_prime = x * (1 + (radialCoefficients[0] * r2) + (radialCoefficients[1] * r4));
      y_prime = y * (1 + (radialCoefficients[0] * r2) + (radialCoefficients[1] * r4));

  // Convert (x,y) to original image co-ordinates (x', y')
  
       x = x_prime * focalLength + xc;
      y = y_prime * focalLength + yc;
  
  // Interpolate
  no.uib.cipr.matrix.Vector p = new DenseVector(new double[] {x, y});
  
  // Write color information
  color c = getColorBilinear(image, p);
  result.set(xi, yi, c);
 
    }
  }

  return result;
}

color getColorBilinear(PImage source, no.uib.cipr.matrix.Vector p) {
  int R, G, B;
  //  Compute the R,G,B pixel values for image coordinates p in source image
  //  using bilinear interpolation


  
  double x = p.get(X);
  double y = p.get(Y);

  int i = (int) Math.floor(x);
  int j = (int) Math.floor(y);

  double a = x - i;
  double b = y - j;

  no.uib.cipr.matrix.Vector w = new DenseVector(new double[] {
    (1-a)*(1-b), a*(1-b), a*b, (1-a)*b,
  }
  );

  color ij = source.get(i, j);
  color iplus1j = source.get(i+1, j);
  color ijplus1 = source.get(i, j+1);
  color iplus1jplus1 = source.get(i+1, j+1);

  Matrix colors = new DenseMatrix(new double[][] {
    { 
      red(ij), red(iplus1j), red(iplus1jplus1), red(ijplus1)
    }
    , 
    { 
      green(ij), green(iplus1j), green(iplus1jplus1), green(ijplus1)
    }
    , 
    { 
      blue(ij), blue(iplus1j), blue(iplus1jplus1), blue(ijplus1)
    }
    ,
  }
  );

  no.uib.cipr.matrix.Vector newColor = new DenseVector(3);
  colors.mult(w, newColor);

  R = round((float) newColor.get(0));
  G = round((float) newColor.get(1));
  B = round((float) newColor.get(2));



  return color(R, G, B);
}




///
/// Runs the Lucas-Kanade motion estimation algorithm
///
/// @param left first image
/// @param right second image
/// @param levels number of levels in the pyramid
/// @param steps number of iterations to run at each level
/// @param translation initial translation guess (see @post)
/// @post parameter translation has been updated to reflect L-K motion estimation
///
void lucasKanade(PImage image1, PImage image2, int levels, int steps, DenseVector translation) {
  
  // Build the pyramids
  PImage[] pyramid1 = getPyramid(image1, levels);
  PImage[] pyramid2 = getPyramid(image2, levels);
  
  // initial translation reduced to smallest level
  translation = translation.scale(1.0/pow(levels-1,2));

  // Loop through pyramids from coarse to fine
  for (int i = levels-1; i >= 0; i--) {
    // Run iterative Lucas Kanade steps times
    for (int j = 0; j < steps; j++) {
      lucasKanadeStep(pyramid1[i], pyramid2[i], translation);
       }
    // Initialize the starting translation vector at the next level by scaling it
    translation = translation.scale(2);
  }
  translation = translation.scale(0.5);
 
}

///
/// Runs a single Lucas-Kanade motion estimation iteration
///
/// @param image1 first image
/// @param image2 second image
/// @param translation initial translation guess (see @post)
/// @post parameter translation has been updated to reflect L-K motion estimation
///
 void lucasKanadeStep(PImage image1,PImage image2,DenseVector translation){
// Translate first by current translation vector
  
  image1 = translate(image1, translation);

  // Build AtA and Atb
  double sumxx = 0;
  double sumyy = 0;
  double sumxy = 0;
  double sumxt = 0;
  double sumyt = 0;

  for (int y = 1; y < image1.height-1; y++) {
    for (int x = 1; x < image1.width-1; x++) {

      double c1_nw = brightness(image1.get(x-1, y-1));
      double c1_n = brightness(image1.get(x, y-1));
      double c1_ne = brightness(image1.get(x+1, y-1));
      double c1_w = brightness(image1.get(x-1, y));
      double c1 = brightness(image1.get(x, y));
      double c1_e = brightness(image1.get(x+1, y));
      double c1_sw = brightness(image1.get(x-1, y+1));
      double c1_s = brightness(image1.get(x, y+1));
      double c1_se = brightness(image1.get(x+1, y+1));

      double c2 = brightness(image2.get(x, y));

    //  double Ix = (c1_ne+c1_e+c1_se-c1_nw-c1_w-c1_sw)/6;
     // double Iy = (c1_sw+c1_s+c1_se-c1_nw-c1_n-c1_ne)/6;
     double Ix = (c1_ne+(2*c1_e)+c1_se-c1_nw-(2*c1_w)-c1_sw)/8;
     double Iy = (c1_ne+(2*c1_n)+c1_nw-c1_se-(2*c1_s)-c1_sw)/8;

      double It = c2-c1;
      sumxx += Ix*Ix;
      sumyy += Iy*Iy;
      sumxy += Ix*Iy;
      sumxt += Ix*It;
      sumyt += Iy*It;
    }
  }

  DenseMatrix AtA = new DenseMatrix(2, 2);
  AtA.set(0, 0, sumxx);
  AtA.set(0, 1, sumxy);
  AtA.set(1, 0, sumxy);
  AtA.set(1, 1, sumyy);

  DenseVector Atb = new DenseVector(2);
  Atb.set(0, -sumxt);
  Atb.set(1, -sumyt);

  // Solve for u,v
  DenseVector uv = new DenseVector(2);
  AtA.solve(Atb, uv);


  // Update translation vector
  translation.set(X, translation.get(X) + uv.get(X));
  translation.set(Y, translation.get(Y) + uv.get(Y));
 }


///
/// Builds a Gaussian pyramid
///
/// @param image full resolution (base) image
/// @param levels number of levels (including base) to return
/// @returns Gaussian pyramid (index 0 is base)
///
PImage[] getPyramid(PImage image, int levels) {
  
 // println("I AM IN GETPYRAMID");
      PImage[] pyramid = new PImage[levels];
      pyramid[0]=copyImage(image);        
  
     for(int i=1;i<levels;i++)
    {
      
      PImage py1 = copyImage(pyramid[i-1]);
      py1.filter(BLUR,2);
      py1.resize(py1.width/2,py1.height/2);  
      pyramid[i]=py1;  
  }     
//println("I AM EXITTING GETPYRAMID");  
      return pyramid; 
   
  
}


///
/// Creates a panorama from the list of ordered images and relative translations
///
/// @param images sequential list of images
///
PImage makePanorama(PImage[] images, DenseMatrix translations) {

  //
  // Do the blending (call blendimages)
  //
  println("LUCAS KANADE finished");
  // start with the last image
  PImage lastImage = images[images.length - 1];
  PImage panoramaImage = createImage(lastImage.width, lastImage.height, RGB);
  


 DenseVector totalTranslations = new DenseVector(2);
 
  for (int i = 0; i < images.length; i++) {
    DenseVector L = new DenseVector(new double[] {
      translations.get(i, 0), translations.get(i, 1)
    }
    );
    totalTranslations.add(L);
  }
  // Vertical drift per column
    slope =  (totalTranslations.get(Y) / totalTranslations.get(X));
  
  for (int h = 0; h < panoramaImage.width; h++) {
    float ih = (float)slope * h;
    for (int w = 0; w < panoramaImage.height; w++) {
      color c1 = lastImage.get(h, (int)floor(w + ih));
      color c2 = lastImage.get(h, (int)floor(w + ih) + 1);

      no.uib.cipr.matrix.Vector source_color = new DenseVector(new double[] {red(c2), green(c2), blue(c2)}, false);
      no.uib.cipr.matrix.Vector target_color = new DenseVector(new double[] {red(c1), green(c1), blue(c1)}, false);
      
      float alpha = (w + ih) - ((int)floor(w + ih)) ;
      no.uib.cipr.matrix.Vector tmp = source_color.scale(alpha).add(target_color.scale(1.0 - alpha));
      color c = color((int)tmp.get(0), (int)tmp.get(1), (int)tmp.get(2));
            
      panoramaImage.set(h, w, c);
    }
  }

  println("Performing Blending... Your image will come soon.");
  totalTranslations = new DenseVector(2);
    for (int i = 0; i < images.length; i++) {
    DenseVector L = new DenseVector(new double[] {
      translations.get(i, 0), translations.get(i, 1)
    }
    );
    totalTranslations.add(L);
  
    panoramaImage = blendImages(panoramaImage, images[i], totalTranslations, 100);
   
  }
 

  
  //
  // Crop the images
  //
  int correctheight = (int)Math.ceil(Math.abs(totalTranslations.get(Y)));
  PImage cropimg = createImage(panoramaImage.width - lastImage.width, panoramaImage.height - correctheight, RGB);
  
  println("Not yet done... Cropping images");
  for (int h = 0; h < panoramaImage.height - correctheight; h++) {
    for (int w = 0; w < panoramaImage.width - lastImage.width; w++) {
      cropimg.set(w, h, panoramaImage.get(w + (lastImage.width / 2), h + correctheight));
    }
  }
    
  return cropimg;
}




//
///
/// Linearly blends two images with a specified window width
///
/// @param image1 first image
/// @param image2 second image
/// @param translation amount to translate second image
/// @param width width of blending bar
/// @returns a new image that's a linear blend of the two images passed
///          the blending bar should be 'width' pixels wide
///
PImage blendImages(PImage image1, PImage image2, DenseVector translation, int width) {

  //
  // Fill in this function such that it linearly blends image and image translated by translation with a window width width
  //
  
  
  int offsetX = (int)translation.get(X);
  PImage result = createImage(offsetX + image2.width, image2.height, RGB);
  
  int translatedX = (int) Math.floor((translation.get(X) + image1.width) / 2.0);
  //
  // Account for drift (do the global warp which involves inverse warping)
  //
  for (int h = 0; h < result.height; h++) {
    // first image
    for (int w = 0; w < translatedX; w++) {
      result.set(w, h, image1.get(w, h));
    }
    
    // second image
    for (int w = translatedX + width; w < result.width; w++) {
        double x2 = w - translation.get(X);
      double y2 = h - translation.get(Y) + slope * w;    
      
      no.uib.cipr.matrix.Vector c2_vector = new DenseVector(new double[] {
        x2, y2
      }
      , false);
int c = getColorBilinear(image2, c2_vector);
result.set(w, h, c);
    }

    //---ALPHA BLENDING---
        for (int dx = 0; dx < width; dx++) {      
      color c1 = image1.get((translatedX + dx), h);
      double x_2 = (translatedX + dx) - translation.get(X);
      double y_2 = (h - translation.get(Y) + slope * (translatedX + dx));
   
      
      no.uib.cipr.matrix.Vector C2_vector = new DenseVector(new double[] {
        x_2, y_2
      }
      , false);
      color c2 = getColorBilinear(image2, C2_vector);
      no.uib.cipr.matrix.Vector source_color =  new DenseVector(new double[] {red(c2), green(c2), blue(c2)}, false);
      no.uib.cipr.matrix.Vector target_color = new DenseVector(new double[] {red(image1.get((translatedX + dx), h)), green(image1.get((translatedX + dx), h)), blue(image1.get((translatedX + dx), h))}); 
      
      double alpha = (dx + 0.5) / width ;
      
      // C= [alpha * Front] + [(1-alpha) * Back]
      no.uib.cipr.matrix.Vector result2 = source_color.scale(alpha).add(target_color.scale(1.0 - alpha));
      color c = color((int)result2.get(0), (int)result2.get(1), (int)result2.get(2));
      result.set((translatedX + dx), h, c);

    }
  }

  return result;
}





//
// -- Some utility functions ----
//



///
/// Translates an image by a vector
///
PImage translate(PImage i, DenseVector translation) {
  int tx = (int)Math.round(translation.get(X));
  int ty = (int)Math.round(translation.get(Y));
  PImage translated =
    createImage(i.width + max(0, tx), i.height + max(0, ty), RGB);

  for(int row = 0; row < i.height; row++){
    for(int column = 0; column < i.width; column++){
      int x = column + tx;
      int y = row + ty;
      translated.set(x, y, i.get(column, row));
    }
  }

  return translated;
}

///
/// Copies an image
///
/// @param i Image to copy
/// @returns a new image with the same pixels as 'i'
///
PImage copyImage(PImage i) {
  PImage newImage = createImage(i.width, i.height, RGB);
  newImage.copy(i, 0, 0, i.width, i.height, 0, 0, i.width, i.height);
  return newImage;
}
