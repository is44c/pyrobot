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
  delete [] workspace;
  delete [] history;
}

PyObject *Vision::initialize(int wi, int he, int de, int r, int g, int b) {
  width = wi;
  height = he;
  depth = de;
  int rgb_order[MAXDEPTH] = {r, g, b};
  for (int d = 0; d < depth; d++)
    // set offsets for RGB
    rgb[d] = rgb_order[d];
  image = new unsigned char [width * height * depth];
  workspace = new unsigned char [width * height * depth];
  history = new unsigned char [width * height * depth];
  memset(image, 0, width * height * depth);
  memset(history, 0, width * height * depth);
  memset(workspace, 0, width * height * depth);
  filterList = PyList_New(0);
  // set the current image to:
  Image = image;
  return PyInt_FromLong(0L);
}

PyObject *Vision::setImage(int newImage) {
  if (newImage == HISTORY)
    Image = history;
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
	brightness += (int) (Image[(h * width + w) * depth + rgb[d]] * weight[rgb[d]]);
	// blacken other pixels:
	Image[(h * width + w) * depth + rgb[d]] = 0;
      }
      if (brightness > 0)
	// reset outChannel pixel to brightness level:
	Image[(h * width + w) * depth + rgb[outChannel] ] = brightness; 
    }
  }
  return PyInt_FromLong(0L);
}  

// match() - match pixels by tolerance

PyObject *Vision::match(int r, int g, int b, int tolerance, 
			int outChannel, int mode) {
  return matchRange( r - tolerance, g - tolerance, b - tolerance,
		     r + tolerance, g + tolerance, b + tolerance,
		     outChannel, mode);
}

// match() - match pixels by range
// outChannel can be RED, GREEN, BLUE, or ALL
// mode is either AND, OR, XOR, or ACCUM

PyObject *Vision::matchRange(int lr, int lg, int lb, 
			     int hr, int hg, int hb,
			     int outChannel, int mode ) {
  int matches;
  for(int h=0;h<height;h++) {
    for(int w=0;w<width;w++) {
      matches = 0;
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
	if (outChannel == ALL) {
	  for (int d = 0; d < depth; d++) {
	    Image[(h * width + w) * depth + rgb[d] ] = 255;
	  }
	} else {
	  for (int d = 0; d < depth; d++) {
	    Image[(h * width + w) * depth + rgb[d] ] = 0;
	  }
	  Image[(h * width + w) * depth + rgb[outChannel] ] = 255;
	}
      } else { // no match
	// leave alone for now
      }
    }
  }
  return Py_BuildValue("");
}

PyObject *Vision::saveImage(char *filename) {
  int w, h;
  int i, j;
  unsigned char *p;
  FILE *fptr;
  
  p=Image;  
  
  if ((fptr=fopen(filename, "w+"))==NULL)
    {
      PyErr_SetString(PyExc_TypeError, 
		      "Unable to open file");
      return NULL;
    }
  else
    {
      fprintf(fptr, "P3\n%d %d\n 255\n", w, h);
      
      for (i=0; i<h; i++) {
	for (j=0; j<w; j++) {
	  fprintf(fptr, "%d %d %d\n", *(p+2), *(p+1), *p); 
	  p += 3;  
	}
      }
      fprintf(fptr,"\n");
      fclose(fptr);
    }
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

PyObject *Vision::colorHistogram() {
  return Py_BuildValue("");
}

PyObject *Vision::trainColor() {
  return Py_BuildValue("");
}

PyObject *Vision::gaussianBlur() {
  int offset,m1,m2;
  int x,y,temp;
  unsigned char *out;
  
  /******************************
    here it says Image[(x-1)+(y+1)*width] 
    if it is changed to Image[(x-depth+offset)+(y+depth)*width]
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
       in this case we want to be *Image, so I think all
       those parts can be omitted
  ******************************************/

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
	Image[(x+offset+y*width)*depth+offset] = temp;
      }
  return PyInt_FromLong(0L);
}

PyObject *Vision::grayScale(int value) {
  int x, y, d;
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
  return PyInt_FromLong(0L);
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


void Vision::sortBlobs(int sortMethod, Blob bloblist[], int indexes[], int size)
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
  int **blobdata;
  blobdata = new int*[width];
  for (i = 0; i < width; i++)
    blobdata[i] = new int[height];

  //[240][384]={0};
  
  printf("Starting blobify...\n");

  unsigned char *ImagePtr;
  PyObject *tuple;
  
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

  printf("Starting blobify 2...\n");
  /*build the blobmap and construct unjoined Blob objects*/
  for(h=0;h<height;h++)
    {
      for(w=0;w<width;w++,ImagePtr+=3)
	{
	  if(*(ImagePtr+offset) >= low && *(ImagePtr+offset) <= high )
	    {  
	      printf("matching pixel at (%d, %d)\n", w, h); // 62, 0
	      if(h == 0 && w == 0)
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
		  else /*above is off -- new blob*/
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
		  else  /* left is off -- new blob */
		    {
		      printf("adding new blob count = %d...", count);
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
		  else /* neither left or top on. -- new blob.*/
		    {
		      initBlob(&bloblist[count],h,w);		      
		      blobdata[w][h]=count;		      
		      count++;
		    }
		}
	    }
	}
    }

  sortBlobs(sortmethod, bloblist, maxIndex,size);

  ImagePtr = Image;

  if(drawBox)
    {
      for(i=0; i<height; i++ )
	for(j=0; j<width; j++,ImagePtr+=3 )
	  for(k=0;k<size;k++) {
	    if(blobdata[j][i] == maxIndex[k]) {
	      *(Image+offset) = high;
	      *(Image+mark1) = 0;
	      *(Image+mark2) = 0;
	    }
	    if(bloblist[maxIndex[k]].mass > 0 )
	      if(((j >= bloblist[maxIndex[k]].ul.x && j <= bloblist[maxIndex[k]].lr.x) &&
		  (i == bloblist[maxIndex[k]].ul.y || i == bloblist[maxIndex[k]].lr.y)) ||
		 ((j == bloblist[maxIndex[k]].ul.x || j == bloblist[maxIndex[k]].lr.x) &&
		  (i >= bloblist[maxIndex[k]].ul.y && i <= bloblist[maxIndex[k]].lr.y)))
		{
		  *(Image+offset) = 255;
		  *(Image+mark1) = 255;
		  *(Image+mark2) = 255;
		}
	  }
    }      
 

  switch(size)
    {
    case 1:
      tuple = Py_BuildValue("iiiii",bloblist[maxIndex[0]].ul.x,
			    bloblist[maxIndex[0]].ul.y,bloblist[maxIndex[0]].lr.x,
			    bloblist[maxIndex[0]].ul.x,bloblist[maxIndex[0]].mass);
      break;
	   
    case 2:
      tuple = Py_BuildValue("iiiiiiiiii",bloblist[maxIndex[0]].ul.x,
			    bloblist[maxIndex[0]].ul.y,bloblist[maxIndex[0]].lr.x,
			    bloblist[maxIndex[0]].lr.y,bloblist[maxIndex[0]].mass,
			    bloblist[maxIndex[1]].ul.x,
			    bloblist[maxIndex[1]].ul.y,bloblist[maxIndex[1]].lr.x,
			    bloblist[maxIndex[1]].lr.y,bloblist[maxIndex[1]].mass);
      break;
    
    case 3:
      tuple = Py_BuildValue("iiiiiiiiiiiiiii",bloblist[maxIndex[0]].ul.x,
			    bloblist[maxIndex[0]].ul.y,bloblist[maxIndex[0]].lr.x,
			    bloblist[maxIndex[0]].lr.y,bloblist[maxIndex[0]].mass,
			    bloblist[maxIndex[1]].ul.x,bloblist[maxIndex[1]].ul.y,
			    bloblist[maxIndex[1]].lr.x,bloblist[maxIndex[1]].lr.y,
			    bloblist[maxIndex[1]].mass,
			    bloblist[maxIndex[2]].ul.x,bloblist[maxIndex[2]].ul.y,
			    bloblist[maxIndex[2]].lr.x,bloblist[maxIndex[2]].lr.y,
			    bloblist[maxIndex[2]].mass);
      break;

    case 4:
      tuple = Py_BuildValue("iiiiiiiiiiiiiiiiiiii",bloblist[maxIndex[0]].ul.x,
			    bloblist[maxIndex[0]].ul.y,bloblist[maxIndex[0]].lr.x,
			    bloblist[maxIndex[0]].lr.y,bloblist[maxIndex[0]].mass,
			    bloblist[maxIndex[1]].ul.x,bloblist[maxIndex[1]].ul.y,
			    bloblist[maxIndex[1]].lr.x,bloblist[maxIndex[1]].lr.y,
			    bloblist[maxIndex[1]].mass,
			    bloblist[maxIndex[2]].ul.x,bloblist[maxIndex[2]].ul.y,
			    bloblist[maxIndex[2]].lr.x,bloblist[maxIndex[2]].lr.y,
			    bloblist[maxIndex[2]].mass,
			    bloblist[maxIndex[3]].ul.x,bloblist[maxIndex[3]].ul.y,
			    bloblist[maxIndex[3]].lr.x,bloblist[maxIndex[3]].lr.y,
			    bloblist[maxIndex[3]].mass);

      break;

    case 5:
      tuple = Py_BuildValue("iiiiiiiiiiiiiiiiiiiiiiiii",bloblist[maxIndex[0]].ul.x,
			    bloblist[maxIndex[0]].ul.y,bloblist[maxIndex[0]].lr.x,
			    bloblist[maxIndex[0]].lr.y,bloblist[maxIndex[0]].mass,
			    bloblist[maxIndex[1]].ul.x,bloblist[maxIndex[1]].ul.y,
			    bloblist[maxIndex[1]].lr.x,bloblist[maxIndex[1]].lr.y,
			    bloblist[maxIndex[1]].mass,
			    bloblist[maxIndex[2]].ul.x,bloblist[maxIndex[2]].ul.y,
			    bloblist[maxIndex[2]].lr.x,bloblist[maxIndex[2]].lr.y,
			    bloblist[maxIndex[2]].mass,
			    bloblist[maxIndex[3]].ul.x,bloblist[maxIndex[3]].ul.y,
			    bloblist[maxIndex[3]].lr.x,bloblist[maxIndex[3]].lr.y,
			    bloblist[maxIndex[3]].mass,
			    bloblist[maxIndex[4]].ul.x,bloblist[maxIndex[4]].ul.y,
			    bloblist[maxIndex[4]].lr.x,bloblist[maxIndex[4]].lr.y,
			    bloblist[maxIndex[4]].mass);
      break;
    }
  /* got blob coordinates, return them. */  
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

PyObject *Vision::applyFilterList() {
  return applyFilters(filterList);
}

PyObject *Vision::applyFilters(PyObject *newList) {
  int i1, i2, i3, i4, i5, i6;
  float f1, f2, f3, f4, f5, f6;
  PyObject *command;
  PyObject *filter, *list;

  if (!PyList_Check(newList)) {
    PyErr_SetString(PyExc_TypeError, "Invalid list to applyFilters");
    return NULL;
  }
  PyObject *retvals = PyList_New( PyList_Size(newList) );
  for (int i = 0; i < PyList_Size(newList); i++) {
    filter = PyList_GetItem(newList, i);
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
      PyList_SetItem(retvals, i, superColor(f1, f2, f3, i1));
    } else if (strcmp((char *)command, "meanBlur") == 0) {
      i1 = 3;
      if (!PyArg_ParseTuple(list, "|i", &i1)) {
	PyErr_SetString(PyExc_TypeError, "Invalid applyFilters: meanBlur");
	return NULL;
      }
      PyList_SetItem(retvals, i, meanBlur(i1));
    } else if (strcmp((char *)command, "sobel") == 0) {
      i1 = 1;
      if (!PyArg_ParseTuple(list, "|i", &i1)) {
	PyErr_SetString(PyExc_TypeError, "Invalid applyFilters: sobel");
	return NULL;
      }
      PyList_SetItem(retvals, i, sobel(i1));
    } else if (strcmp((char *)command, "setPlane") == 0) {
      i1 = 0; i2 = 0;
      if (!PyArg_ParseTuple(list, "|ii", &i1, &i2)) {
	PyErr_SetString(PyExc_TypeError, "Invalid applyFilters: setPlane");
	return NULL;
      }
      PyList_SetItem(retvals, i, setPlane(i1, i2));
    } else if (strcmp((char *)command, "drawRect") == 0) {
      i1 = 0; i2 = 0; i3 = 0; i4 = 0; i5 = 0; i6 = ALL;
      if (!PyArg_ParseTuple(list, "|iiiiii", &i1, &i2, &i3, &i4, &i5, &i6)) {
	PyErr_SetString(PyExc_TypeError, "Invalid applyFilters: drawRect");
	return NULL;
      }
      PyList_SetItem(retvals, i, drawRect(i1, i2, i3, i4, i5, i6));
    } else if (strcmp((char *)command, "match") == 0) {
      i1 = 0; i2 = 0; i3 = 0; i4 = 30; i5 = 0; i6 = ACCUM;
      if (!PyArg_ParseTuple(list, "|iiiiii", &i1, &i2, &i3, &i4, &i5, &i6)) {
	PyErr_SetString(PyExc_TypeError, "Invalid applyFilters: match()");
	return NULL;
      }
      PyList_SetItem(retvals, i, match(i1, i2, i3, i4, i5, i6));
    } else if (strcmp((char *)command, "grayScale") == 0) {
      i1 = 255;
      if (!PyArg_ParseTuple(list, "|i", &i1)) {
	PyErr_SetString(PyExc_TypeError, "Invalid applyFilters: grayScale");
	return NULL;
      }
      PyList_SetItem(retvals, i, grayScale(i1));
    } else if (strcmp((char *)command, "threshold") == 0) {
      i1 = 0; i2 = 200;
      if (!PyArg_ParseTuple(list, "|ii", &i1, &i2)) {
	PyErr_SetString(PyExc_TypeError, "Invalid applyFilters: threshold");
	return NULL;
      }
      PyList_SetItem(retvals, i, threshold(i1, i2));
    } else if (strcmp((char *)command, "inverse") == 0) {
      i1 = 0;
      if (!PyArg_ParseTuple(list, "|i", &i1)) {
	PyErr_SetString(PyExc_TypeError, "Invalid applyFilters: inverse");
	return NULL;
      }
      PyList_SetItem(retvals, i, inverse(i1));
    } else {
      PyErr_SetString(PyExc_TypeError, "Invalid command to applyFilters");
      return NULL;
    }
  }

  return retvals;
}
