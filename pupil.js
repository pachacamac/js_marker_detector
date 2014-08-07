var AR = AR || {};

AR.Marker = function(id, corners){
  this.id = id;
  this.corners = corners;
};

AR.logArray = function(a){
  var s = '';
  for(i = 0; i < a.length; ++ i){
    for(j = 0; j < a[i].length; ++ j) s += a[i][j]+" ";
    s += "\n";
  }
  console.log(s);
};

AR.Marker.generate = function(id, s){
  // Marker is based on grid of black (0) and white (1) pixels. First bit is encoded in the orientation corners of the marker
  // b|b|b|b|b                                   MSB = 0          MSB = 1
  // b|o|m|o|b  b = black border feature         W|*|W   ^        B|*|B   ^
  // b|m|m|m|b  o = orientation feature          *|*|*   | UP     *|*|*   | UP
  // b|o|m|o|b  m = message feature              B|*|W   |        W|*|B   |
  // b|b|b|b|b

  // IDEA: encode 2 bits in orientation feature?
  //    11  00  10  00
  //    01  10  01  11

  var size = s || 5;
  var m=[], msg=[], msb, bitdepth, i, j;

  bitdepth = Math.pow(size-2, 2)-3; // bitdepth = (size-border)^2 - 3 for orientation (orientation still yields one bit)
  if(id >= Math.pow(2, bitdepth)){throw new Error("ID overflow! This marker can only hold "+bitdepth+" bits of information");}

  for(i=0; i<size; i++) {
    m[i] = new Array();
    for(j=0; j<size; j++) m[i][j] = 0;
  }

  msg = new Array(bitdepth);
  for(i=bitdepth-1; i>=0; i--){
    msg[i] = id % 2;
    id = id >> 1;
  }

  msb = msg.shift();
  for(i=1; i<size-1; i++){
    for(j=1; j<size-1; j++){
      if(i==1&&j==1 || i==size-2&&j==size-2 || i==1&&j==size-2){
        m[i][j] = 1 - msb;
      }else if(i==size-2&&j==1){
        m[i][j] = msb;
      }else{
        m[i][j] = msg.shift();
      }
    }
  }

  AR.logArray(m);
  return m;
};

AR.Detector = function(debug_elem){
  this.grey = new CV.Image();
  this.thres = new CV.Image();
  this.homography = new CV.Image();
  this.binary = [];
  this.contours = [];
  this.polys = [];
  this.candidates = [];
  this.debug_elem = debug_elem;
};

AR.Detector.prototype.detect = function(image){
  CV.grayscale(image, this.grey);
  CV.adaptiveThreshold(this.grey, this.thres, 4, 7);

  this.contours = CV.findContours(this.thres, this.binary);

  this.candidates = this.findCandidates(this.contours, image.width * 0.20, 0.05, 10);
  this.candidates = this.clockwiseCorners(this.candidates);
  this.candidates = this.notTooNear(this.candidates, 10);
  return this.findMarkers(this.grey, this.candidates, 49);
};

AR.Detector.prototype.findCandidates = function(contours, minSize, epsilon, minLength){
  var candidates = [], len = contours.length, contour, poly, i;
  this.polys = [];
  for (i = 0; i < len; ++ i){
    contour = contours[i];
    if (contour.length >= minSize){
      poly = CV.approxPolyDP(contour, contour.length * epsilon);
      this.polys.push(poly);
      if ( (4 === poly.length) && ( CV.isContourConvex(poly) ) ){
        if ( CV.minEdgeLength(poly) >= minLength){
          candidates.push(poly);
        }
      }
    }
  }
  return candidates;
};

AR.Detector.prototype.clockwiseCorners = function(candidates){
  var len = candidates.length, dx1, dx2, dy1, dy2, swap, i;
  for (i = 0; i < len; ++ i){
    dx1 = candidates[i][1].x - candidates[i][0].x;
    dy1 = candidates[i][1].y - candidates[i][0].y;
    dx2 = candidates[i][2].x - candidates[i][0].x;
    dy2 = candidates[i][2].y - candidates[i][0].y;
    if ( (dx1 * dy2 - dy1 * dx2) < 0){
      swap = candidates[i][1];
      candidates[i][1] = candidates[i][3];
      candidates[i][3] = swap;
    }
  }
  return candidates;
};

AR.Detector.prototype.notTooNear = function(candidates, minDist){
  var notTooNear = [], len = candidates.length, dist, dx, dy, i, j, k;
  for (i = 0; i < len; ++ i){
    for (j = i + 1; j < len; ++ j){
      dist = 0;
      for (k = 0; k < 4; ++ k){
        dx = candidates[i][k].x - candidates[j][k].x;
        dy = candidates[i][k].y - candidates[j][k].y;
        dist += dx * dx + dy * dy;
      }
      if ( (dist / 4) < (minDist * minDist) ){
        if ( CV.perimeter( candidates[i] ) < CV.perimeter( candidates[j] ) ){
          candidates[i].tooNear = true;
        }else{
          candidates[j].tooNear = true;
        }
      }
    }
  }
  for (i = 0; i < len; ++ i){
    if ( !candidates[i].tooNear ){
      notTooNear.push( candidates[i] );
    }
  }
  return notTooNear;
};

AR.Detector.prototype.findMarkers = function(imageSrc, candidates, warpSize){
  var markers = [], len = candidates.length, candidate, marker, i;
  for (i = 0; i < len; ++ i){
    candidate = candidates[i];
    CV.warp(imageSrc, this.homography, candidate, warpSize);
    CV.threshold(this.homography, this.homography, CV.otsu(this.homography) );
    marker = this.getMarker(this.homography, candidate);
    if (marker){
      markers.push(marker);
    }
  }
  return markers;
};

// TODO: make marker grid-capacity dynamic!
AR.Detector.prototype.getMarker = function(imageSrc, candidate){
  var width = (imageSrc.width / 5) >>> 0,
      minZero = (width * width) >> 1,
      bits = [], corners = [], distances = [],
      corner_sum, msg_int, rotations, square, pair, inc, i, j;

  for (i = 0; i < 5; ++ i){
    inc = (0 === i || 4 === i)? 1: 4;
    for (j = 0; j < 5; j += inc){
      square = {x: j * width, y: i * width, width: width, height: width};
      if ( CV.countNonZero(imageSrc, square) > minZero){
        return null;
      }
    }
  }

  for (i = 0; i < 3; ++ i){
    bits[i] = [];
    for (j = 0; j < 3; ++ j){
      square = {x: (j + 1) * width, y: (i + 1) * width, width: width, height: width};
      bits[i][j] = CV.countNonZero(imageSrc, square) > minZero? 1: 0;
    }
  }

  corners = [bits[0][0], bits[0][2], bits[2][2], bits[2][0]]
  corner_sum = corners[0] + corners[1] + corners[2] + corners[3];
  if(corner_sum == 3){
    msg_int = 0;
  }else if(corner_sum == 1){
    msg_int = 1;
    //invert bits, make sure we have 3 white and 1 black
    corners = [1 - corners[0], 1 - corners[1], 1 - corners[2], 1 - corners[3]];
  }else{
    return null; //invalid marker!
  }

  // determine number of 90deg rotations to normalize marker.
  if     (corners[0] == 0) rotations = 3;
  else if(corners[3] == 0) rotations = 0;
  else if(corners[2] == 0) rotations = 1;
  else                     rotations = 2;


  //console.log("rotations: "+rotations);
  // normalize marker
  AR.logArray(bits);
  for(i=0; i<rotations; i++){
    bits = this.rotate(bits);
  }
  AR.logArray(bits);
  //bits = this.transpose(bits);

  // convert bits to ID
  msg_int = (msg_int<<1) + bits[0][1];
  msg_int = (msg_int<<1) + bits[1][0];
  msg_int = (msg_int<<1) + bits[1][1];
  msg_int = (msg_int<<1) + bits[1][2];
  msg_int = (msg_int<<1) + bits[2][1];

  return new AR.Marker(msg_int, this.rotate2(candidate, rotations) );
};

AR.Detector.prototype.rotate = function(arr){
  var dst = [], len = arr.length, i, j;
  for (i = 0; i < len; ++ i){
    dst[i] = [];
    for (j = 0; j < arr[i].length; ++ j){
      dst[i][j] = arr[arr[i].length - j - 1][i];
    }
  }
  return dst;
};

AR.Detector.prototype.transpose = function(arr){
  return arr[0].map(function(col, i) {
    return arr.map(function(row) {
      return row[i]
    })
  });
};

AR.Detector.prototype.rotate2 = function(src, rotation){
  var dst = [], len = src.length, i;
  for (i = 0; i < len; ++ i){
    dst[i] = src[ (rotation + i) % len ];
  }
  return dst;
};
