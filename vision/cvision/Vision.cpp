#include "Vision.h"

Vision::Vision() {
}

Vision::Vision(int wi, int he, int de, int r, int g, int b) {
  initialize(wi, he, de, r, g, b);
}

Vision::Vision(int wi, int he, int de) {
  initialize(wi, he, de, 0, 1, 2);
}

Vision::~Vision() {
  delete [] image;
}

PyObject *Vision::registerCameraDevice(void *dev) {
  Device *device = (Device *)dev;
  image = device->getImage();
  return initialize(device->getWidth(), device->getHeight(), device->getDepth(),
		    device->getRGB()[0],device->getRGB()[1],device->getRGB()[2]);
}

PyObject *Vision::getRGB() {
  return Py_BuildValue("iii", rgb[0], rgb[1], rgb[2] );
}


PyObject *Vision::setRGB(int r, int g, int b) {
  int rgb_order[MAXDEPTH] = {r, g, b};
  for (int d = 0; d < depth; d++)
    // set offsets for RGB
    rgb[d] = rgb_order[d];
  return getRGB();
}

PyObject *Vision::initialize(int wi, int he, int de, int r, int g, int b) {
  width = wi;
  height = he;
  depth = de;
  setRGB(r, g, b);
  filterList = PyList_New(0);
  // set the current image to:
  Image = image;
  return PyInt_FromLong(0L);
}

PyObject *Vision::setImage(int newImage) {
  if (newImage == ORIGINAL)
    Image = original;
  else if (newImage == IMAGE) 
    Image = image;
  else if (newImage == WORKSPACE)
    Image = workspace;
  else {
    PyErr_SetString(PyExc_ValueError, "invalid image ID in setImage()");
    return NULL;
  }
  return Py_BuildValue("");
}

// set(): works on current image
// sets the depth R, G, or B at (w, h) to val.
// if d is RED, GREEN, or BLUE then set just that d; if
// d == ALL, then set all colors to val.

PyObject *Vision::set(int w, int h, int r, int g, int b) {
  set(w, h, rgb[0], r);
  set(w, h, rgb[1], g);
  set(w, h, rgb[2], b);
  return Py_BuildValue("i", 0);
}

PyObject *Vision::set(int w, int h, int d, int value) {
  if (w < 0 || w >= width ||
      h < 0 || h >= height ||
      value < 0 || value >= 256) {
    PyErr_SetString(PyExc_ValueError, "width, height, or value out-of-bounds in set()");
    return NULL;
  } else {
    if (d < MAXDEPTH)
      Image[(h * width + w) * depth + rgb[d]] = value;
    else if (d == ALL)
      for (int deep = 0; deep < depth; deep++)
	Image[(h * width + w) * depth + deep] = value;
    else {
      PyErr_SetString(PyExc_ValueError, "invalid color in set()");
      return NULL;
    }
  }
  return Py_BuildValue("i", value);
}

// same as set(), but for the entire color plane (depth)

PyObject *Vision::setPlane(int d, int value) {
  for (int w=0; w<width; w++) {
    for (int h=0; h<height; h++) {
      if (d < MAXDEPTH)
	Image[(h * width + w) * depth + rgb[d]] = value;
      else if (d == ALL)
	for (int deep = 0; deep < depth; deep++)
	  Image[(h * width + w) * depth + deep] = value;
      else {
	PyErr_SetString(PyExc_ValueError, "invalid color in set()");
	return NULL;
      }
    }
  }
  return Py_BuildValue("i", value);
}

PyObject *Vision::scale(float r, float g, float b) {
  for (int w=0; w<width; w++) {
    for (int h=0; h<height; h++) {
      for (int d=0; d<depth; d++) {
	Image[(h * width + w) * depth + rgb[0]] = (int)(MIN(MAX(r * Image[(h * width + w) * depth + rgb[0]],0),255));
	Image[(h * width + w) * depth + rgb[1]] = (int)(MIN(MAX(g * Image[(h * width + w) * depth + rgb[1]],0),255));
	Image[(h * width + w) * depth + rgb[2]] = (int)(MIN(MAX(b * Image[(h * width + w) * depth + rgb[2]],0),255));
      }
    }
  }
  return PyInt_FromLong(0L);
}

PyObject *Vision::getMMap() {
  PyObject *buffer;
  buffer = PyBuffer_FromMemory(Image, 
			       width * height * depth * sizeof(unsigned char));
  return Py_BuildValue("O", buffer);
}


PyObject *Vision::get(int w, int h) {
  if (w < 0 || w >= width ||
      h < 0 || h >= height) {
    PyErr_SetString(PyExc_ValueError, "width or height out-of-bounds in get");
    return NULL;
  }
  if (depth == 3) {
    return Py_BuildValue("iii", 
			 Image[(h * width + w) * depth + rgb[0]],
			 Image[(h * width + w) * depth + rgb[1]],
			 Image[(h * width + w) * depth + rgb[2]] );
  } else if (depth == 1) {
    return Py_BuildValue("i", Image[(h * width + w) * depth + 0]);
  } else {
    PyErr_SetString(PyExc_ValueError, "Invalid depth in get");
    return NULL;
  }
}

PyObject *Vision::superColor(float w1, float w2, float w3, 
			int outChannel) {
  float weight[3] = {w1, w2, w3};
  for (int w=0; w<width; w++) {
    for (int h=0; h<height; h++) {
      int brightness = 0;
      for (int d = 0; d < depth; d++) {
	// compute brightness as sum of values * weight
	brightness += (int) (Image[(h * width + w) * depth + rgb[d]] * weight[d]);
	// blacken all pixels:
	Image[(h * width + w) * depth + rgb[d]] = 0;
      }
      if (brightness > 0)
	// reset outChannel pixel to brightness level:
	Image[(h * width + w) * depth + rgb[outChannel] ] = MAX(MIN(brightness,255),0); 
    }
  }
  return PyInt_FromLong(0L);
}  

// match() - match pixels by tolerance

PyObject *Vision::match(int r, int g, int b, int tolerance, 
			int outChannel) {
  return matchRange( r - tolerance, g - tolerance, b - tolerance,
		     r + tolerance, g + tolerance, b + tolerance,
		     outChannel);
}

// match() - match pixels by range
// outChannel can be RED, GREEN, BLUE, or ALL
// mode is either AND, OR, XOR, or ACCUM

PyObject *Vision::matchRange(int lr, int lg, int lb, 
			     int hr, int hg, int hb,
			     int outChannel) {
  int matches = 0;
  for(int h=0;h<height;h++) {
    for(int w=0;w<width;w++) {
      if (
	  (lr == -1 || 
	   (Image[(h * width + w) * depth + rgb[RED]] >= lr && 
	    Image[(h * width + w) * depth + rgb[RED]] <= hr))
	  &&
	  (lg == -1 || 
	   (Image[(h * width + w) * depth + rgb[GREEN]] >= lg && 
	    Image[(h * width + w) * depth + rgb[GREEN]] <= hg))
	  &&
	  (lb == -1 ||
	   (Image[(h * width + w) * depth + rgb[BLUE]] >=  lb && 
	    Image[(h * width + w) * depth + rgb[BLUE]] <= hb))
	  ) {
	/* maybe add a normalizer here so the outputs are 
	   between 100-255ish for more varied usage? */
	matches++;
	if (outChannel == ALL) {
	  for (int d = 0; d < depth; d++) {
	    Image[(h * width + w) * depth + d ] = 255;
	  }
	} else {
	  for (int d = 0; d < depth; d++) {
	    Image[(h * width + w) * depth + d ] = 0;
	  }
	  Image[(h * width + w) * depth + rgb[outChannel] ] = 255;
	}
      } else { // no match
	// leave alone for now
      }
    }
  }
  return Py_BuildValue("i", matches);
}

PyObject *Vision::saveImage(char *filename) {
  int i, j;
  FILE *fptr;
  if (0 != rgb[0])
    swapPlanes(0, rgb[0]);
  if ((fptr=fopen(filename, "wb+"))==NULL)
    {
      PyErr_SetString(PyExc_TypeError, 
		      "Unable to open file");
      return NULL;
    }
  else
    {
      fprintf(fptr, "P6\n%d %d\n 255\n", width, height);
      fwrite(Image, (width * height * depth), 1, fptr);
      fclose(fptr);
    }
  if (0 != rgb[0])
    swapPlanes(0, rgb[0]);
  return Py_BuildValue("");
} 

PyObject *Vision::drawRect(int x1, int y1, int x2, int y2, 
		      int fill, int channel) {
  for(int w=x1; w<=x2; w++) {
      for(int h=y1; h<=y2; h++ ) {
	if (fill == 1 || (h == x1 || h == x2 ||
			  w == y1 || w == y2))
	  if (channel == ALL)
	    for(int d=0; d<depth; d++) {
	      Image[(h * width + w) * depth + d] = 255;
	    }
	  else
	    Image[(h * width + w) * depth + rgb[channel]] = 255;
      }
  }
  return PyInt_FromLong(0L);
}

PyObject *Vision::histogram(int x1, int y1, int x2, int y2, int bins) {
  int b, d;
  long int **binCnt;
  long int **binAvg;
  long int *maxAvg;
  long int *maxCnt;
  // DON'T ASSUME THAT bin count won't change. Reallocate these.
  // OR make a maximum size, and just leave them here.
  binCnt = new long int *[depth];
  binAvg = new long int *[depth];
  maxAvg = new long int[depth];
  maxCnt = new long int[depth];
  for (d = 0; d < depth; d++) {
    binCnt[d] = new long int[bins];
    binAvg[d] = new long int[bins];
    maxAvg[d] = 0;
    maxCnt[d] = 0;
    for (b = 0; b < bins; b++) {
      binCnt[d][b] = 0;
      binAvg[d][b] = 0;
    }
  }
  // First, make a histogram:
  for (int w = x1; w <= x2; w++) {
    for (int h = y1; h <= y2; h++) {
      for (int d = 0; d < depth; d++) {
	int bin = int(Image[(h * width + w) * depth + d] / 256.0 * bins);
	binAvg[d][ bin ] += Image[(h * width + w) * depth + d];
	binCnt[d][ bin ]++;
      }
    }
  }
  // Compute avg, and remember best:
  for (d = 0; d < depth; d++) {
    for (b = 0; b < bins; b++) {
      if (binCnt[d][b]) {
	binAvg[d][b] /= binCnt[d][b];
      }
      if (binCnt[d][b] > maxCnt[d]) {
	maxAvg[d] = binAvg[d][b];
	maxCnt[d] = binCnt[d][b];
      }
    }
  }
  PyObject *histList = PyList_New( bins );
  for (int i = 0; i < bins; i++) {
    PyList_SetItem(histList, i, Py_BuildValue("(iii)(iii)", 
					      binAvg[rgb[0]][i], 
					      binAvg[rgb[1]][i], 
					      binAvg[rgb[2]][i],
					      binCnt[rgb[0]][i], 
					      binCnt[rgb[1]][i], 
					      binCnt[rgb[2]][i] ));
  }
  PyObject *retval = Py_BuildValue("O(iii)", 
				   histList,
				   maxAvg[rgb[0]],
				   maxAvg[rgb[1]],
				   maxAvg[rgb[2]] );
  // clean up
  for (d = 0; d < depth; d++) {
    delete [] binCnt[d];
    delete [] binAvg[d];
  }
  delete [] binCnt;
  delete [] binAvg;
  delete [] maxAvg;
  delete [] maxCnt;
  return retval;
}

PyObject *Vision::grayScale() {
  int x, y, d, value;
  for (y=0; y<height; y++)
    for(x=0; x<width; x++)
      {
	value = 0;
	for (d = 0; d < depth; d++) {
	  value += (int)Image[(x+y*width)*depth + d];
	}
	value /= depth;
	for (d = 0; d < depth; d++) {
	  Image[(x+y*width)*depth + d]= value;
	}
      }
  return PyInt_FromLong(0L);
}

PyObject *Vision::threshold(int channel, int value) {
  int x, y;
  for (y=0; y<height; y++)
    for(x=0; x<width; x++) {
      if (Image[(x+y*width)*depth + rgb[channel]] >= value) {
	Image[(x+y*width)*depth + rgb[channel]] = 255;
      } else {
	Image[(x+y*width)*depth + rgb[channel]] = 0;
      }
    }
  return Py_BuildValue("");
}

PyObject *Vision::inverse(int channel) {
  int x, y;
  for (y=0; y<height; y++)
    for(x=0; x<width; x++) {
      Image[(x+y*width)*depth + rgb[channel]] = 255 - Image[(x+y*width)*depth + rgb[channel]];
    }
  return Py_BuildValue("");
}

PyObject *Vision::sobel(int thresh) {
  int i, j, offset;
  unsigned int tempx, tempy;
  unsigned char *ImagePtr;
  unsigned char *out;
  
  unsigned int a,b,d,f,g,z,c,e,h,gc, sobscale;

  out = (unsigned char *)malloc(sizeof(char)*width*height*depth);
    
  ImagePtr = Image;
  
  for (j=0;j<height*width*depth;j++)
    out[j]=0;

  offset = 0;
  sobscale = 1;

  i = j = 0;
  
  for (j=0;j<height-2;j++)
  {
    a = Image[(j*width+i)*depth];
    b = Image[(j*width+(i+1))*depth];
    d = Image[((j+1)*width+i)*depth];
    f = Image[((j+2)*width+i)*depth];
    g = Image[((j+2)*width+(i+1))*depth];
    z = Image[(j*width+i)*depth];
    
    for (i=0;i<width-2;i++) 
    {
      c = Image[(j*width+(i+2))*depth];
      e = Image[((j+1)*width+(i+2))*depth];
      h = Image[((j+2)*width+(i+2))*depth];

      tempx = (a+d+f) - (c+e+h);
      if( tempx < 0 ) tempx = -1*tempx;
      
      tempy = (a+b+c) - (f+g+h);
      if( tempy < 0 ) tempy = -1*tempy;

      gc = (unsigned int) (sobscale * sqrt(tempx*tempx+tempy*tempy));
      gc = offset+gc;
      gc = (gc>255)? 0 : 255 - gc;

      out[(j*width+i)*depth] = gc;
      out[(j*width+i)*depth+1] = gc;
      out[(j*width+i)*depth+2] = gc;

      a=b;
      b=c;
      d=z;
      f=g;
      g=h;
      z=e;

    }
    i=0;

  }

  ImagePtr = Image;
  for (j=0;j<height;j++)
    for (i=0;i<width;i++)
      for(offset=0;offset<depth;offset++)
	Image[(i+j*width)*depth+offset] = out[(i+j*width)*depth+offset] ;

  free(out);
 
  return PyInt_FromLong(0L);
}

PyObject *Vision::medianBlur(int kernel) {
  int mid;
  int w,h,i,j,x, moveVal, offset, temp=0;
  int median[4][400] = {{-1}};  /*enough for 20x20(huge) Kernel */
  int intensity = 3;
  unsigned char *ptr, *outptr, *imagePtr;
  static unsigned char *out = new unsigned char[width * height * depth];

  if(kernel <= 0)
    kernel = 1;
  else if(kernel % 2 == 0) 
    kernel--;
  
  x=(int)kernel/2;
  
  moveVal = x*width+x;

  imagePtr = Image+(moveVal)*3;
  outptr = out+(moveVal)*3;
  
  for (j=0;j<height*width*depth;j++)
    out[j]=0;

  offset = 0;

  for(h=x;h<height-x;h++)
    {
      for(w=x;w<width-x;w++,imagePtr+=3,outptr+=3)
	{
	  ptr=imagePtr-(moveVal)*3;
	  temp=0;
	 
	  /* find middle color of surrouding pixels */
	  for(i=0;i<kernel;i++)
	    {
	      for(j=0;j<kernel;j++,ptr+=3,temp++)
		{
		  median[BLUE][temp]  = *(ptr);
		  median[GREEN][temp] = *(ptr+1);
		  median[RED][temp]  = *(ptr+2);
		  median[intensity][temp]=(int) (0.3*(*(ptr+2)) + 0.59*(*(ptr+1)) + 0.11*(*(ptr)));
		}
	      /* bring ptr to next row */
	      ptr=ptr-(kernel*3);
	      ptr=ptr+width*3; /* bring ptr to next row */
	    }
	  
	  /* get median intensity index */
	  mid=getMiddleIndex(median, kernel);

	  *outptr = median[BLUE][mid];
	  *(outptr+1) = median[GREEN][mid];
	  *(outptr+2) = median[RED][mid];

	}
    }

  imagePtr = Image;
  outptr = out;
  for (j=0;j<height;j++)
    for (i=0;i<width;i++)
      for(offset=0;offset<depth;offset++)
	Image[(i+j*width)*depth+offset] = outptr[(i+j*width)*depth+offset] ;

  return PyInt_FromLong(0L);
}

int Vision::getMiddleIndex(int median[4][400], int kernel) {
  int i,j;
  int rankTable[400];
  
  for(i=0; i<kernel*kernel; i++)
    rankTable[i] = i;
  
  for(i=0; i < kernel*kernel; i++)
    for(j=i+1; j< kernel*kernel; j++)
      if(median[3][rankTable[i]] > median[3][rankTable[j]])
	SWAP(rankTable[i],rankTable[j]);
  
  return(rankTable[(int)((kernel*kernel)/2)]);
}

PyObject *Vision::meanBlur(int kernel) {
  int w,h,i,j,side,d;
  unsigned int average[3]={0};
  
  if(kernel <= 0)
    kernel = 1;
  else if(kernel % 2 == 0) 
    kernel--;
  
  side=(int)kernel/2;
  
  for(h=side;h<height-side;h++)
    {
      for(w=side;w<width-side;w++)
	{
	  /* calculate average color of surrounding pixels*/
	  for(i=-side;i<=side;i++)
	    for(j=-side;j<=side;j++)
	      for(d=0;d<depth;d++)
		average[d] += Image[((h+i)* width + (w + j)) * depth + d];
	  
	  
	  for(d=0;d<depth;d++)
	    Image[(h * width + w) * depth + d] = average[d] / (kernel*kernel);
	  
	  average[0] = 0;
	  average[1] = 0;
	  average[2] = 0;
	}
    }
  return PyInt_FromLong(0L);
}

int Vision::getWidth() { return width; }
int Vision::getHeight() { return height; }
int Vision::getDepth() { return depth; }


// ----------------------- Blob Functions ------------------

Blob *Vision::initBlob(Blob *b) {
  b->mass = 0;
  b->ul.x = width;
  b->ul.y = height;
  b->lr.x = 0;
  b->lr.y = 0;
  b->cm.x = 0;
  b->cm.y = 0;
  b->next = 0;  
  return(b);
}

Blob *Vision::initBlob( Blob *b, int y, int x )
{
  b->mass = 1;
  b->ul.x=x;
  b->ul.y=y;
  b->lr.x=x;
  b->lr.y=y;
  b->cm.x=x;
  b->cm.y=y;
  b->next = 0;
  return (b);
}

Blob *Vision::addPixel( Blob *b, int y,int x )
{
  if( x < b->ul.x )
    b->ul.x = x;  
  if( x > b->lr.x )
    b->lr.x = x;
  if( y < b->ul.y )
    b->ul.y = y;
  if( y > b->lr.y )
    b->lr.y = y;
  /* not correct */
  /*b->cm.x =( (float)(b->mass * b->cm.x + x) / (float)(b->mass+1) );
    b->cm.y =( (float)(b->mass * b->cm.y + y) / (float)(b->mass+1) );*/
  b->mass++;
  return (b);
}

void Vision::joinBlob( Blob *self, Blob *other )
{
  if(self->mass != 0 && other->mass != 0)
    {
      if( other->ul.x < self->ul.x )
	self->ul.x = other->ul.x;
      
      if( other->lr.x > self->lr.x )
	self->lr.x = other->lr.x ;
      
      if( other->ul.y < self->ul.y )
	self->ul.y = other->ul.y;
      
      if( other->lr.y > self->lr.y )
	self->lr.y = other->lr.y;
      
      /* Not Correct */
      /*
	self->cm.x=( (self->mass * self->cm.x + other->mass * other->cm.x )/
	(self->mass + other->mass));
	self->cm.y=( (self->mass * self->cm.y + other->mass * other->cm.y)/
	(self->mass + other->mass));
      */ 
      self->mass += other->mass;      
      other->mass = 0;
    }
}

void Vision::deleteBlob( Blob *b )
{
  
  b->cm.x = width / 2;
  b->cm.y = height / 2;
  b->ul.x = width;
  b->ul.y = height;
  b->lr.x = 0;
  b->lr.x = 0;
  b->mass = 0;
  
}

int Vision::getBlobWidth( Blob *b )
{
  return( b->lr.x - b->ul.x );
}

int Vision::getBlobHeight( Blob *b )
{
  return( b->lr.y - b->ul.y );
}

int Vision::getBlobArea( Blob *b )
{
  return( getBlobWidth( b ) * getBlobHeight( b ) );
}

/* not correct, 1 pixel is very dense.  */
/*
  float getBlobDensity( Blob *b )
  {
  return( (float)b->mass / (float)getBlobArea( b ) );
  }
*/


void Vision::sortBlobs(int sortMethod, Blob bloblist[], 
		       int indexes[], int size)
{
  int i,j;
  int rankTable[MAXBLOBS];
  
  
  for(i=0; i<MAXBLOBS; i++)
    rankTable[i] = i;
  
  switch(sortMethod)
    {
      
    case 0:/* Mass */
      for(i=1; i < size+1; i++)
	for(j=i+1; j< MAXBLOBS; j++)
	  if(bloblist[rankTable[i]].mass < bloblist[rankTable[j]].mass)
	    SWAP(rankTable[i],rankTable[j]);
      break;
      
    case 1: /* Area */
      for(i=1; i < size+1; i++)
	for(j=i+1; j< MAXBLOBS; j++)
	  /* automattically swap out 0 mass from i spot*/
	  if(bloblist[rankTable[i]].mass == 0 ||
	     (bloblist[rankTable[j]].mass != 0 &&
	      getBlobArea(&bloblist[rankTable[i]]) <
	      getBlobArea(&bloblist[rankTable[j]])))
	    SWAP(rankTable[i],rankTable[j]); 
      break;
    
    }
  
  for(i=1;i<size+1;i++)
    indexes[i-1]= rankTable[i];
  
}

PyObject *Vision::blobify(int inChannel, int low, int high, 
			  int sortmethod, 
			  int size, int drawBox)
{
  /** 
      This code is a way to find the largest blob.  It will 
      combine blobs if there is more than one.  However, given
      the colors that we are using in the project, it is 
      unlikely that we'll have more than one blob with the 
      same color (unless the same object is split into multiple 
      regions by some other object)
      
      The function works by looking at the top and left pixel
      to see if there is already a blob there.  It joins or adds
      the pixel to the blob, and merges blobs if they are connected.
  **/
  
  Blob bloblist[MAXBLOBS];
  
  int w,h,n,m,i,j, k, l;
  int offset, mark1, mark2;
  int count;
  int minBlobNum=0, maxBlobNum=0;

  int maxIndex[5]={0};
  static int **blobdata;
  static int initialized = 0;
  if (!initialized) {
    // ASSUMES height and width don't change!
    blobdata = new int*[width];
    for (i = 0; i < width; i++) {
      blobdata[i] = new int[height];
    }
    initialized = 1;
  }
  // Always need to initilize to zero
  for (i = 0; i < width; i++) {
    for (j = 0; j < height; j++)
      blobdata[i][j] = 0;
  }
  unsigned char *ImagePtr;
  
  if(inChannel == BLUE)
    {
      offset = rgb[2]; mark1 = rgb[0]; mark2 = rgb[1];
    }
  else if(inChannel == GREEN)
    {
      offset = rgb[1]; mark1 = rgb[0]; mark2 = rgb[2];
    }
  else if(inChannel == RED)
    {
      offset = rgb[0]; mark1 = rgb[1]; mark2 = rgb[2];
    }
  else
    perror("Invalid Channel\n");
  
  
  for(n=0;n<MAXBLOBS;n++)
    initBlob(&bloblist[n]);
  
  count = 1;
  
  ImagePtr = Image;

  /*build the blobmap and construct unjoined Blob objects*/
  for(h=0;h<height;h++)
    {
      for(w=0;w<width;w++,ImagePtr+=3)
	{
	  if(*(ImagePtr+offset) >= low && *(ImagePtr+offset) <= high )
	    {  
	      if(h == 0 && w == 0 && count < MAXBLOBS)
		{ /*in upper left corner - new blob */
		  initBlob(&bloblist[count],h,w);
		  blobdata[w][h]= count;
		  count++;
		}
	      else if(w == 0)/*if in first col */
		{
		  if( blobdata[w][h-1] != 0 )
		    {
		      addPixel(&bloblist[blobdata[w][h-1]],h,w);
		      blobdata[w][h]=blobdata[w][h-1];
		    }
		  else if (count < MAXBLOBS) /*above is off -- new blob*/
		    {
		      initBlob(&bloblist[count], h,w);		      
		      blobdata[w][h]=count;
		      count++;
		    }
		}
	      else if(h == 0) /* in first row */
		{
		  if( blobdata[w-1][h] != 0 )
		    {
		      addPixel(&bloblist[blobdata[w-1][h]],h,w);
		      blobdata[w][h]= blobdata[w-1][h];
		    }
		  else if (count < MAXBLOBS)  /* left is off -- new blob */
		    {
		      initBlob(&bloblist[count], h,w);
		      blobdata[w][h]=count;
		      count++;
		    }		    
		}
	      
	      else if( blobdata[w-1][h] != 0 && blobdata[w][h-1] != 0 )
		{
		  /*
		    see if the pixel to left and on the top are the same blob and add 
		    this new pixel to the blob if they are 
		  */
		  if(blobdata[w-1][h] == blobdata[w][h-1])
		    {
		      addPixel(&bloblist[blobdata[w-1][h]],h,w);
		      blobdata[w][h] = blobdata[w-1][h];
		    }
		  else 
		    {
		      addPixel(&bloblist[blobdata[w-1][h]],h,w);		      
		      joinBlob(&bloblist[blobdata[w-1][h]],&bloblist[blobdata[w][h-1]]);
		      blobdata[w][h] = blobdata[w-1][h];
		      
		      n = blobdata[w][h-1];		      
		      for(i=0;i<=h;i++)
			for(j=0;j<width;j++)
			  if(blobdata[j][i] == n)
			    blobdata[j][i] = blobdata[w-1][h];
		      
		      /*deleteBlob(&bloblist[blobdata[w][h-1]]);*/
		    }
		}
	      else
		{
		  if( blobdata[w-1][h] != 0 )
		    {
		      addPixel(&bloblist[blobdata[w-1][h]],h,w);		  
		      blobdata[w][h]=blobdata[w-1][h];		      
		    }
		  /*top is on -- old blob */
		  else if( blobdata[w][h-1] != 0 )
		    {		      
		      addPixel(&bloblist[blobdata[w][h-1]],h,w);
		      blobdata[w][h]=blobdata[w][h-1];
		    }
		  else if (count < MAXBLOBS) /* neither left or top on. -- new blob.*/
		    {
		      initBlob(&bloblist[count],h,w);		      
		      blobdata[w][h]=count;		      
		      count++;
		    }
		}
	    }
	}
    }

  sortBlobs(sortmethod, bloblist, maxIndex, size);

  if(drawBox)
    {
      for(i=0; i<height; i++ )
	for(j=0; j<width; j++)
	  for(k=0;k<size;k++) {
	    if(blobdata[j][i] == maxIndex[k]) {
	      Image[(i * width + j) * depth + offset] = 0;
	      Image[(i * width + j) * depth + mark1] = 255;
	      Image[(i * width + j) * depth + mark2] = 0;
	    }
	    if(bloblist[maxIndex[k]].mass > 0 )
	      if(((j >= bloblist[maxIndex[k]].ul.x && j <= bloblist[maxIndex[k]].lr.x) &&
		  (i == bloblist[maxIndex[k]].ul.y || i == bloblist[maxIndex[k]].lr.y)) ||
		 ((j == bloblist[maxIndex[k]].ul.x || j == bloblist[maxIndex[k]].lr.x) &&
		  (i >= bloblist[maxIndex[k]].ul.y && i <= bloblist[maxIndex[k]].lr.y)))
		{
		  Image[(i * width + j) * depth + offset] = 255;
		  Image[(i * width + j) * depth + mark1] = 255;
		  Image[(i * width + j) * depth + mark2] = 255;
		}
	  }
    }      
  
  PyObject *tuple = PyTuple_New( size );

  for (i = 0; i < size; i++) {
    PyTuple_SetItem(tuple, i, 
		    Py_BuildValue("iiiii",
				  bloblist[maxIndex[i]].ul.x,
				  bloblist[maxIndex[i]].ul.y,
				  bloblist[maxIndex[i]].lr.x,
				  bloblist[maxIndex[i]].ul.x,
				  bloblist[maxIndex[i]].mass));
  }
  return tuple; 
}  

PyObject *Vision::getFilterList() {
  Py_INCREF(filterList);
  return filterList;
}

PyObject *Vision::setFilterList(PyObject *newList) {
  if (!PyList_Check(newList)) {
    PyErr_SetString(PyExc_TypeError, "Invalid list to setFilters");
    return NULL;
  }
  Py_DECREF(filterList);
  filterList = newList;
  Py_INCREF(filterList);
  return Py_BuildValue("");
}

PyObject *Vision::popFilterList() {
  int size = PyList_Size(filterList);
  if (size > 0) {
    PyObject *retval = PyList_GetItem(filterList, size - 1); 
    Py_INCREF(retval);
    PyList_SetSlice(filterList, size - 1, size, Py_BuildValue("[]"));
    return retval;
  } else {
    PyErr_SetString(PyExc_TypeError, "can't remove items from an empty list in popFilterList");
    return NULL;
  }
}

PyObject *Vision::addFilter(PyObject *newFilter) {
  //if (!PyList_Check(newFilter)) {
  //  PyErr_SetString(PyExc_TypeError, "Invalid filter to addFilter");
  //  return NULL;
  //}
  Py_INCREF(newFilter);
  PyList_Append(filterList, newFilter);
  return Py_BuildValue("");
}

PyObject *Vision::gaussianBlur() {
  int offset,m1,m2;
  int x,y,temp;

  /******************************
    here it says image[(x-1)+(y+1)*width] 
    if it is changed to image[(x-depth+offset)+(y+depth)*width]
    it should work for rgb (or bgr as the case may be)
    -----------------------------------------------------
     this means we will probably need another for loop for
     offset, as we probably want to blur for all channels
     ---------------------------------------------------
       Get each of the surrounding pixels weighted value
       top, left, right, and bottom weighing 2x
       diagonals weighing 1x
       pixel itself weighing 4x
       and take the average
       --------------------------------------------
       I think *out is just the output buffer, which
       in this case we want to be *image, so I think all
       those parts can be omitted
  ******************************************/
  static unsigned char *out = new unsigned char [width * height * depth];
  for (y=1;y<height-1;y++)
    for (x=1;x<width-1;x++)
      for(offset=0;offset<depth;offset++)
      {
	temp=Image[(x-depth+offset+y*width)*depth+offset];
	temp+=2*Image[(x-depth+y*width)*depth+offset];
	temp+=Image[(x-depth+(y+1)*width)*depth+offset];
	temp+=2*Image[(x+(y-1)*width)*depth+offset];
	temp+=4*Image[(x+y*width)*depth+offset];
	temp+=2*Image[(x+(y+1)*width)*depth+offset];
	temp+=Image[(x+depth+offset+(y-1)*width)*depth+offset];
	temp+=2*Image[(x+depth+offset+y*width)*depth+offset];
	temp+=Image[(x+depth+offset+(y+1)*width)*depth+offset];
	temp/=16;
	out[(x+offset+y*width)*depth+offset] = temp;
      }

  for (y=1;y<height-1;y++)
    for (x=1;x<width-1;x++)
      for(offset=0;offset<depth;offset++)
      	Image[(x+y*width)*depth+offset] = out[(x+y*width)*depth+offset] ;
  
  return PyInt_FromLong(0L);

} 

PyObject *Vision::applyFilterList() {
  return applyFilters(filterList);
}

/*
PyObject *suppliesFilters() {
  // ("text name", menu?, funcName, (**defaultArgs)),
  (("super color", 0, "superColor" , (1, -1, -1, 0)),
   ("clear plane", 0, "setPlane", (0, 0),
    ("draw rectangle", 0, "drawRect", (0, 0, 0, 0, 0, ALL)),
    ("match", 0, "match", (0, 0, 0, 30, 0, ACCUM)),
    ("threshold", 0, "threshold", (0, 200)),
    ("inverse", 0, "inverse", (0,)),
    ("blur edges", 1, "meanBlur", (3)),
    ("detect edges", 1, "sobel", (1) ),
    ("gray scale", 1, "grayScale", ()),
    ("blobify", 1, "blobify", (0)),                          
    ("clear red", 1, "setPlane", (0),
    ("clear green", 1, "setPlane", (1),
    ("clear blue", 1, "setPlane", (2),                          
    ("superColor red", 1, "superColor", (1, -1, -1, 0),
    ("superColor green", 1, "superColor", (-1, 1, -1, 1),
    ("superColor blue", 1, "superColor", (-1, -1, 1, 2),
    ("threshold red", 1, "threshold", (0),
    ("threshold green", 1, "threshold", (1),
    ("threshold blue", 1, "threshold", (2),
    ("inverse red", 1, "inverse", (0),                          
    ("inverse green", 1, "inverse", (1),
    ("inverse blue", 1, "inverse", (2)
}
*/

PyObject *Vision::backup() { 
  return copy(0); // 0 backup, 1 restore
}

PyObject *Vision::restore() { 
  return copy(1); // 0 backup, 1 restore
}

PyObject *Vision::motion() { 
  static unsigned char *motion = new unsigned char[width * height * depth];
  static unsigned char *temp = new unsigned char[width * height * depth];
  int threshold = 30;
  for (int w = 0; w < width; w++) {
    for (int h = 0; h < height; h++) {
      int totalDiff = 0;
      for (int d = 0; d < depth; d++) {
	totalDiff += abs(Image[(h * width + w) * depth + d] - motion[(h * width + w) * depth + d]);
      }
      for (int d = 0; d < depth; d++) {
	if (totalDiff/3 > threshold)
	  temp[(h * width + w) * depth + d] = 255;
	else
	  temp[(h * width + w) * depth + d] = 0;
      }
    }
  }
  memcpy(motion, Image, width * height * depth);
  memcpy(Image, temp, width * height * depth);
  return Py_BuildValue("");
}

PyObject *Vision::copy(int fromto) { // 0 backup, 1 restore
  static unsigned char *backup = new unsigned char[width * height * depth];
  if (fromto == 0) { // backup
    memcpy(backup, Image, width * height * depth);
  } else if (fromto == 1) { // restore
    memcpy(Image, backup, width * height * depth);
  }
  return PyInt_FromLong(0L);
}

PyObject *Vision::applyFilter(PyObject *filter) {
  int i1, i2, i3, i4, i5, i6, i7;
  float f1, f2, f3, f4, f5, f6, f7;
  PyObject *command, *list, *retval;
    if (!PyArg_ParseTuple(filter, "s|O", &command, &list)) {
      PyErr_SetString(PyExc_TypeError, "Invalid filter list name to applyFilters");
      return NULL;
    }
    // process filters here:
    if (strcmp((char *)command, "superColor") == 0) {
      f1 = 1.0, f2 = -1.0, f3 = -1.0, i1 = 0;
      if (!PyArg_ParseTuple(list, "|fffi", &f1, &f2, &f3, &i1)) {
	PyErr_SetString(PyExc_TypeError, "Invalid applyFilters: superColor");
	return NULL;
      }
      retval = superColor(f1, f2, f3, i1);
    } else if (strcmp((char *)command, "scale") == 0) {
      f1 = 1.0, f2 = 1.0, f3 = 1.0;
      if (!PyArg_ParseTuple(list, "|fff", &f1, &f2, &f3)) {
	PyErr_SetString(PyExc_TypeError, "Invalid applyFilters: scale()");
	return NULL;
      }
      retval = scale(f1, f2, f3);
    } else if (strcmp((char *)command, "meanBlur") == 0) {
      i1 = 3;
      if (!PyArg_ParseTuple(list, "|i", &i1)) {
	PyErr_SetString(PyExc_TypeError, "Invalid applyFilters: meanBlur");
	return NULL;
      }
      retval = meanBlur(i1);
    } else if (strcmp((char *)command, "medianBlur") == 0) {
      i1 = 3;
      if (!PyArg_ParseTuple(list, "|i", &i1)) {
	PyErr_SetString(PyExc_TypeError, "Invalid applyFilters: medianBlur");
	return NULL;
      }
      retval = medianBlur(i1);
    } else if (strcmp((char *)command, "gaussianBlur") == 0) {
      retval = gaussianBlur();
    } else if (strcmp((char *)command, "sobel") == 0) {
      i1 = 1;
      if (!PyArg_ParseTuple(list, "|i", &i1)) {
	PyErr_SetString(PyExc_TypeError, "Invalid applyFilters: sobel");
	return NULL;
      }
      retval = sobel(i1);
    } else if (strcmp((char *)command, "setPlane") == 0) {
      i1 = 0; i2 = 0;
      if (!PyArg_ParseTuple(list, "|ii", &i1, &i2)) {
	PyErr_SetString(PyExc_TypeError, "Invalid applyFilters: setPlane");
	return NULL;
      }
      retval = setPlane(i1, i2);
    } else if (strcmp((char *)command, "set") == 0) {
      i1 = 0; i2 = 0, i3 = 255, i4 = 255, i5 = 255;
      if (!PyArg_ParseTuple(list, "|iiiii", &i1, &i2, &i3, &i4, &i5)) {
	PyErr_SetString(PyExc_TypeError, "Invalid applyFilters: setPlane");
	return NULL;
      }
      retval = set(i1, i2, i3, i4, i5);
    } else if (strcmp((char *)command, "drawRect") == 0) {
      i1 = 0; i2 = 0; i3 = 0; i4 = 0; i5 = 0; i6 = ALL;
      if (!PyArg_ParseTuple(list, "|iiiiii", &i1, &i2, &i3, &i4, &i5, &i6)) {
	PyErr_SetString(PyExc_TypeError, "Invalid applyFilters: drawRect");
	return NULL;
      }
      retval = drawRect(i1, i2, i3, i4, i5, i6);
    } else if (strcmp((char *)command, "match") == 0) {
      i1 = 0; i2 = 0; i3 = 0; i4 = 30; i5 = 0;
      if (!PyArg_ParseTuple(list, "|iiiii", &i1, &i2, &i3, &i4, &i5)) {
	PyErr_SetString(PyExc_TypeError, "Invalid applyFilters: match()");
	return NULL;
      }
      retval = match(i1, i2, i3, i4, i5);
    } else if (strcmp((char *)command, "matchRange") == 0) {
      i1 = 0; i2 = 0; i3 = 0; i4 = 255; i5 = 255; i6 = 255, i7 = 0;
      if (!PyArg_ParseTuple(list, "|iiiiiii", &i1, &i2, &i3, &i4,&i5,&i6,&i7)) {
	PyErr_SetString(PyExc_TypeError, "Invalid applyFilters: matchRange()");
	return NULL;
      }
      retval = matchRange(i1, i2, i3, i4, i5, i6, i7);
    } else if (strcmp((char *)command, "grayScale") == 0) {
      retval = grayScale();
    } else if (strcmp((char *)command, "backup") == 0) {
      retval = backup();
    } else if (strcmp((char *)command, "restore") == 0) {
      retval = restore();
    } else if (strcmp((char *)command, "motion") == 0) {
      retval = motion();
    } else if (strcmp((char *)command, "threshold") == 0) {
      i1 = 0; i2 = 200;
      if (!PyArg_ParseTuple(list, "|ii", &i1, &i2)) {
	PyErr_SetString(PyExc_TypeError, "Invalid applyFilters: threshold");
	return NULL;
      }
      retval = threshold(i1, i2);
    } else if (strcmp((char *)command, "inverse") == 0) {
      i1 = 0;
      if (!PyArg_ParseTuple(list, "|i", &i1)) {
	PyErr_SetString(PyExc_TypeError, "Invalid applyFilters: inverse");
	return NULL;
      }
      retval = inverse(i1);
    } else if (strcmp((char *)command, "blobify") == 0) {
      i1 = 0; i2 = 200; i3 = 255; i4 = 0; i5 = 1; i6 = 1;
      // inChannel, low, high, sortmethod 0 = mass, 1 = area, return blobs, 
      // drawBox
      if (!PyArg_ParseTuple(list, "|iiiiii", &i1, &i2, &i3, &i4, &i5, &i6)) {
	PyErr_SetString(PyExc_TypeError, "Invalid applyFilters: blobify");
	return NULL;
      }
      retval = blobify(i1, i2, i3, i4, i5, i6);
    } else if (strcmp((char *)command, "histogram") == 0) {
      i1 = 0; i2 = 0; i3 = width - 1; i4 = height - 1; i5 = 8;
      // x1, y1, x2, y2, bins
      if (!PyArg_ParseTuple(list, "|iiiii", &i1, &i2, &i3, &i4, &i5)) {
	PyErr_SetString(PyExc_TypeError, "Invalid applyFilters: histogram");
	return NULL;
      }
      retval = histogram(i1, i2, i3, i4, i5);
    } else {
      PyErr_SetString(PyExc_TypeError, "Invalid command to applyFilter");
      return NULL;
    }
    return retval;
}

PyObject *Vision::applyFilters(PyObject *newList) {
  PyObject *filter;
  if (!PyList_Check(newList)) {
    PyErr_SetString(PyExc_TypeError, "Invalid list to applyFilters");
    return NULL;
  }
  PyObject *retvals = PyList_New( PyList_Size(newList) );
  for (int i = 0; i < PyList_Size(newList); i++) {
    filter = PyList_GetItem(newList, i);
    PyList_SetItem(retvals, i, applyFilter( filter ));
  }
  return retvals;
}

void Vision::swapPlanes(int d1, int d2) {
  for (int h = 0; h < height; h++) {
    for (int w = 0; w < width; w++) {
      unsigned int temp = Image[(h * width + w) * depth + d1];
      Image[(h * width + w) * depth + d1] = Image[(h * width + w) * depth + d2];
      Image[(h * width + w) * depth + d2] = temp;
    }
  }
}