/*! docuSnap v3.0.0 | (c) ColonelParrot and other contributors | MIT License */

(function (global, factory) {
  if (typeof exports === "object" && typeof module !== "undefined") {
    module.exports = factory();
  } else if (typeof define === "function" && define.amd) {
    define(factory);
  } else {
    var exported = factory();
    var g = typeof globalThis !== "undefined" ? globalThis
          : typeof self !== "undefined" ? self
          : typeof window !== "undefined" ? window
          : global;
    g.DocuSnap = exported.DocuSnap;
  }
})(this, function () {
  "use strict";

  // =========================================================================
  // QualityAssessor — pure JS image quality metrics (no OpenCV)
  // =========================================================================

  function QualityAssessor() {}

  /**
   * Convert RGBA image data to grayscale Float32Array.
   * Uses ITU-R BT.601 weights: 0.299R + 0.587G + 0.114B
   */
  QualityAssessor.prototype.rgbaToGrayscale = function (rgba, w, h) {
    var count = w * h;
    var gray = new Float32Array(count);
    for (var i = 0; i < count; i++) {
      var ri = i * 4;
      gray[i] = 0.299 * rgba[ri] + 0.587 * rgba[ri + 1] + 0.114 * rgba[ri + 2];
    }
    return gray;
  };

  /**
   * Measures image sharpness using Laplacian variance.
   * Higher values = sharper image. Threshold ~100 recommended.
   * Applies 3x3 Laplacian kernel: [0, 1, 0, 1, -4, 1, 0, 1, 0]
   */
  QualityAssessor.prototype.measureSharpness = function (gray, w, h) {
    var step = 1;
    if (w > 800) {
      step = Math.ceil(w / 640);
    }
    var sw = Math.floor(w / step);
    var sh = Math.floor(h / step);

    var n = 0;
    var sum = 0;
    var sumSq = 0;

    for (var y = 1; y < sh - 1; y++) {
      for (var x = 1; x < sw - 1; x++) {
        var sy = y * step;
        var sx = x * step;

        var center = gray[sy * w + sx];
        var top    = gray[(sy - step) * w + sx];
        var bottom = gray[(sy + step) * w + sx];
        var left   = gray[sy * w + (sx - step)];
        var right  = gray[sy * w + (sx + step)];

        var lap = top + bottom + left + right - 4 * center;

        sum += lap;
        sumSq += lap * lap;
        n++;
      }
    }

    if (n === 0) return 0;
    var mean = sum / n;
    return sumSq / n - mean * mean;
  };

  /**
   * Measures mean brightness of the image.
   */
  QualityAssessor.prototype.measureBrightness = function (gray, w, h) {
    var count = w * h;
    var sum = 0;
    for (var i = 0; i < count; i++) {
      sum += gray[i];
    }
    return sum / count;
  };

  /**
   * Detects glare by measuring fraction of very bright pixels.
   * Uses lower threshold (200) to catch camera glare which often isn't pure white.
   */
  QualityAssessor.prototype.detectGlare = function (gray, w, h, threshold) {
    if (threshold == null) threshold = 200;  // Lowered to catch typical camera glare
    var count = w * h;
    var bright = 0;
    for (var i = 0; i < count; i++) {
      if (gray[i] > threshold) bright++;
    }
    return bright / count;
  };

  /**
   * Detects glare within a quadrilateral region (document bounds).
   * Much more accurate than measuring the whole frame.
   */
  QualityAssessor.prototype.detectGlareInRegion = function (gray, w, h, cornerPoints, threshold) {
    if (threshold == null) threshold = 200;
    if (!cornerPoints) return this.detectGlare(gray, w, h, threshold);
    
    var tl = cornerPoints.topLeftCorner;
    var tr = cornerPoints.topRightCorner;
    var bl = cornerPoints.bottomLeftCorner;
    var br = cornerPoints.bottomRightCorner;
    if (!tl || !tr || !bl || !br) return this.detectGlare(gray, w, h, threshold);
    
    // Get bounding box of the quadrilateral
    var minX = Math.max(0, Math.floor(Math.min(tl.x, tr.x, bl.x, br.x)));
    var maxX = Math.min(w - 1, Math.ceil(Math.max(tl.x, tr.x, bl.x, br.x)));
    var minY = Math.max(0, Math.floor(Math.min(tl.y, tr.y, bl.y, br.y)));
    var maxY = Math.min(h - 1, Math.ceil(Math.max(tl.y, tr.y, bl.y, br.y)));
    
    var count = 0;
    var bright = 0;
    
    // Check pixels within bounding box that are inside the quad
    for (var y = minY; y <= maxY; y++) {
      for (var x = minX; x <= maxX; x++) {
        if (this._pointInQuad(x, y, tl, tr, br, bl)) {
          count++;
          if (gray[y * w + x] > threshold) bright++;
        }
      }
    }
    
    return count > 0 ? bright / count : 0;
  };

  /**
   * Check if point is inside quadrilateral using cross product method.
   */
  QualityAssessor.prototype._pointInQuad = function (px, py, tl, tr, br, bl) {
    // Check if point is on the same side of all 4 edges
    var d1 = (px - tl.x) * (tr.y - tl.y) - (py - tl.y) * (tr.x - tl.x);
    var d2 = (px - tr.x) * (br.y - tr.y) - (py - tr.y) * (br.x - tr.x);
    var d3 = (px - br.x) * (bl.y - br.y) - (py - br.y) * (bl.x - br.x);
    var d4 = (px - bl.x) * (tl.y - bl.y) - (py - bl.y) * (tl.x - bl.x);
    
    var hasNeg = (d1 < 0) || (d2 < 0) || (d3 < 0) || (d4 < 0);
    var hasPos = (d1 > 0) || (d2 > 0) || (d3 > 0) || (d4 > 0);
    
    return !(hasNeg && hasPos);
  };

  // =========================================================================
  // KalmanFilter1D — Simple 1D Kalman filter for smooth position tracking
  // Tracks position and velocity for predictive smoothing
  // =========================================================================

  /**
   * 1D Kalman filter for smooth position tracking.
   * State vector: [position, velocity]
   * @param {number} initialValue - Initial position
   * @param {number} [processNoise=0.1] - Process noise (lower = smoother but slower)
   * @param {number} [measurementNoise=1.0] - Measurement noise (higher = trust predictions more)
   */
  function KalmanFilter1D(initialValue, processNoise, measurementNoise) {
    // State: [position, velocity]
    this.x = initialValue;    // Position estimate
    this.v = 0;               // Velocity estimate
    
    // Covariance matrix (2x2, stored as 4 values: P00, P01, P10, P11)
    this.P00 = 1;
    this.P01 = 0;
    this.P10 = 0;
    this.P11 = 1;
    
    // Noise parameters (tuned for smooth bounding box tracking)
    // Lower Q = smoother, higher R = trust predictions more (less jitter)
    this.Q = processNoise != null ? processNoise : 0.01;     // Process noise (lowered for smoothness)
    this.R = measurementNoise != null ? measurementNoise : 8.0;  // Measurement noise (increased to reduce jitter)
  }

  /**
   * Update filter with new measurement and return filtered position.
   * @param {number} measurement - New measured position
   * @returns {number} Filtered position estimate
   */
  KalmanFilter1D.prototype.update = function(measurement) {
    // --- Predict step ---
    // State prediction: x' = x + v, v' = v (constant velocity model)
    var xPred = this.x + this.v;
    var vPred = this.v;
    
    // Covariance prediction: P' = F*P*F' + Q
    // F = [1 1; 0 1], so:
    // P'00 = P00 + P10 + P01 + P11 + Q
    // P'01 = P01 + P11
    // P'10 = P10 + P11
    // P'11 = P11 + Q
    var P00 = this.P00 + this.P10 + this.P01 + this.P11 + this.Q;
    var P01 = this.P01 + this.P11;
    var P10 = this.P10 + this.P11;
    var P11 = this.P11 + this.Q;
    
    // --- Update step ---
    // Kalman gain: K = P' * H' * (H * P' * H' + R)^-1
    // H = [1 0], so K = [P'00; P'10] / (P'00 + R)
    var S = P00 + this.R;  // Innovation covariance
    var K0 = P00 / S;      // Kalman gain for position
    var K1 = P10 / S;      // Kalman gain for velocity
    
    // Innovation (measurement residual)
    var y = measurement - xPred;
    
    // State update
    this.x = xPred + K0 * y;
    this.v = vPred + K1 * y;
    
    // Covariance update: P = (I - K*H) * P'
    this.P00 = (1 - K0) * P00;
    this.P01 = (1 - K0) * P01;
    this.P10 = -K1 * P00 + P10;
    this.P11 = -K1 * P01 + P11;
    
    return this.x;
  };

  /**
   * Coast: predict-only step with no measurement update.
   * Advances position by last known velocity without correcting toward any measurement.
   * Use this when the detector misses a frame so the box drifts smoothly rather than freezing or disappearing.
   * @returns {number} Predicted position
   */
  KalmanFilter1D.prototype.coast = function() {
    this.x = this.x + this.v;
    var P00 = this.P00 + this.P01 + this.P10 + this.P11 + this.Q;
    var P01 = this.P01 + this.P11;
    var P10 = this.P10 + this.P11;
    var P11 = this.P11 + this.Q;
    this.P00 = P00; this.P01 = P01; this.P10 = P10; this.P11 = P11;
    return this.x;
  };

  /**
   * Reset filter to a new initial value.
   * @param {number} value - New initial position
   */
  KalmanFilter1D.prototype.reset = function(value) {
    this.x = value;
    this.v = 0;
    this.P00 = 1;
    this.P01 = 0;
    this.P10 = 0;
    this.P11 = 1;
  };

  // =========================================================================
  // DocumentDetector — Line-based rectangle detector
  // Pipeline: Sobel edges → Hough lines → perpendicular intersections → rectangles
  // Much faster than contour tracing, and only finds actual rectangles.
  // =========================================================================

  /**
   * @param {object} [options]
   * @param {number} [options.minAspectRatio=1.2] - minimum width/height ratio
   * @param {number} [options.maxAspectRatio=1.8] - maximum width/height ratio
   * @param {number} [options.minWidthFraction=0.40] - min document width as fraction of frame width
   * @param {number} [options.maxAreaFraction=0.85] - max document area as fraction of frame
   * @param {number} [options.processWidth=480] - downscale width for processing speed
   */
  function DocumentDetector(options) {
    options = options || {};
    // Aspect ratio for ID cards (1.586) and passports (1.42) with ~20% tolerance
    this._minAspect = options.minAspectRatio != null ? options.minAspectRatio : 1.2;
    this._maxAspect = options.maxAspectRatio != null ? options.maxAspectRatio : 1.8;
    this._minWidthFraction = options.minWidthFraction != null ? options.minWidthFraction : 0.25;  // 25% minimum (lowered for debugging)
    this._maxAreaFraction = options.maxAreaFraction != null ? options.maxAreaFraction : 0.85;
    this._processWidth = options.processWidth || 480;  // Higher res for better edge detection
    // Pre-compute Hough sin/cos tables
    this._houghReady = false;
    this._sinTable = null;
    this._cosTable = null;
    this._numAngles = 180;
  }

  /** Initialize Hough lookup tables. */
  DocumentDetector.prototype.init = async function () {
    this._sinTable = new Float32Array(this._numAngles);
    this._cosTable = new Float32Array(this._numAngles);
    for (var i = 0; i < this._numAngles; i++) {
      var theta = i * Math.PI / this._numAngles;
      this._sinTable[i] = Math.sin(theta);
      this._cosTable[i] = Math.cos(theta);
    }
    this._houghReady = true;
  };

  /**
   * Detects the largest card-shaped rectangle in the image.
   * @param {Uint8ClampedArray} rgba - RGBA pixel data
   * @param {number} w - image width
   * @param {number} h - image height
   * @returns {object|null} { bbox, confidence, cornerPoints } or null
   */
  DocumentDetector.prototype.detect = function (rgba, w, h) {
    // 1. Downscale for speed
    var scale = 1;
    var pw = w, ph = h;
    if (w > this._processWidth) {
      scale = this._processWidth / w;
      pw = Math.round(w * scale);
      ph = Math.round(h * scale);
    }

    // Get grayscale and downscaled RGBA at processing resolution
    var gray, rgbaSmall;
    if (scale < 1) {
      var result = this._downscaleGrayAndRgba(rgba, w, h, pw, ph);
      gray = result.gray;
      rgbaSmall = result.rgba;
    } else {
      gray = new Uint8Array(w * h);
      rgbaSmall = rgba;
      for (var i = 0; i < w * h; i++) {
        var ri = i * 4;
        gray[i] = Math.round(0.299 * rgba[ri] + 0.587 * rgba[ri + 1] + 0.114 * rgba[ri + 2]);
      }
    }

    // 2. Fast 3x3 Gaussian blur
    var blurred = this._gaussianBlur3x3(gray, pw, ph);

    // 3. Sobel edge detection with gradient magnitude
    var edges = this._sobelEdges(blurred, pw, ph);

    // 4. Create skin mask and suppress edges in skin regions
    // TODO: Skin mask may be too aggressive - disabled for debugging
    // var skinMask = this._createSkinMask(rgbaSmall, pw, ph);
    // this._suppressEdgesInMask(edges, skinMask, pw, ph);

    // 5. Hough line transform on edge pixels
    var lines = this._houghLines(edges, pw, ph);
    if (this.debug) console.log('[docuSnap] lines found:', lines.length, lines.slice(0,8).map(function(l){ return {theta: Math.round(l.theta*180/Math.PI)+'°', rho: Math.round(l.rho), votes: l.votes}; }));
    if (lines.length < 4) { if (this.debug) console.warn('[docuSnap] KILLED: fewer than 4 lines'); return null; }

    // 6. Find rectangle from line intersections (pass edges for support scoring)
    var best = this._findRectangleFromLines(lines, pw, ph, scale, edges);
    if (this.debug && !best) console.warn('[docuSnap] KILLED: no valid quad found (see individual rejections above)');

    return best;
  };

  /** @private - Downscale RGBA to grayscale and keep RGBA for skin detection */
  DocumentDetector.prototype._downscaleGrayAndRgba = function (rgba, srcW, srcH, dstW, dstH) {
    var srcCanvas = document.createElement("canvas");
    srcCanvas.width = srcW;
    srcCanvas.height = srcH;
    var srcCtx = srcCanvas.getContext("2d");
    var imgData = srcCtx.createImageData(srcW, srcH);
    imgData.data.set(rgba);
    srcCtx.putImageData(imgData, 0, 0);

    var dstCanvas = document.createElement("canvas");
    dstCanvas.width = dstW;
    dstCanvas.height = dstH;
    var dstCtx = dstCanvas.getContext("2d");
    dstCtx.drawImage(srcCanvas, 0, 0, dstW, dstH);

    var pixels = dstCtx.getImageData(0, 0, dstW, dstH).data;
    var gray = new Uint8Array(dstW * dstH);
    for (var i = 0; i < dstW * dstH; i++) {
      var ri = i * 4;
      gray[i] = Math.round(0.299 * pixels[ri] + 0.587 * pixels[ri + 1] + 0.114 * pixels[ri + 2]);
    }
    return { gray: gray, rgba: pixels };
  };

  /**
   * @private - Create skin mask using YCrCb color space.
   * Returns Uint8Array where 255 = skin, 0 = not skin.
   */
  DocumentDetector.prototype._createSkinMask = function (rgba, w, h) {
    var mask = new Uint8Array(w * h);
    for (var i = 0; i < w * h; i++) {
      var ri = i * 4;
      var r = rgba[ri], g = rgba[ri + 1], b = rgba[ri + 2];
      
      // Convert RGB to YCrCb
      var y  = 0.299 * r + 0.587 * g + 0.114 * b;
      var cr = (r - y) * 0.713 + 128;
      var cb = (b - y) * 0.564 + 128;
      
      // Skin detection thresholds in YCrCb space
      // Cr: 133-173, Cb: 77-127 (typical skin tones)
      if (cr >= 133 && cr <= 173 && cb >= 77 && cb <= 127 && y > 60) {
        mask[i] = 255;
      }
    }
    return mask;
  };

  /**
   * @private - Suppress edges that fall within the skin mask.
   * This prevents hand/face edges from competing with document edges.
   */
  DocumentDetector.prototype._suppressEdgesInMask = function (edges, mask, w, h) {
    // Dilate skin mask slightly to catch edges at skin boundaries
    var dilated = new Uint8Array(w * h);
    var radius = 2;
    for (var y = radius; y < h - radius; y++) {
      for (var x = radius; x < w - radius; x++) {
        var idx = y * w + x;
        if (mask[idx]) {
          // Mark neighborhood
          for (var dy = -radius; dy <= radius; dy++) {
            for (var dx = -radius; dx <= radius; dx++) {
              dilated[(y + dy) * w + (x + dx)] = 255;
            }
          }
        }
      }
    }
    
    // Suppress edges in dilated skin regions
    for (var i = 0; i < w * h; i++) {
      if (dilated[i]) edges[i] = 0;
    }
  };

  /** @private - Fast 3x3 Gaussian blur (sigma ~0.85) */
  DocumentDetector.prototype._gaussianBlur3x3 = function (gray, w, h) {
    var out = new Uint8Array(w * h);
    // Kernel: [1,2,1; 2,4,2; 1,2,1] / 16
    for (var y = 1; y < h - 1; y++) {
      for (var x = 1; x < w - 1; x++) {
        var val =
          gray[(y-1)*w+(x-1)]     + 2*gray[(y-1)*w+x]     + gray[(y-1)*w+(x+1)] +
          2*gray[y*w+(x-1)]       + 4*gray[y*w+x]         + 2*gray[y*w+(x+1)] +
          gray[(y+1)*w+(x-1)]     + 2*gray[(y+1)*w+x]     + gray[(y+1)*w+(x+1)];
        out[y * w + x] = (val + 8) >> 4;
      }
    }
    // Copy border pixels
    for (var x = 0; x < w; x++) { out[x] = gray[x]; out[(h-1)*w+x] = gray[(h-1)*w+x]; }
    for (var y = 0; y < h; y++) { out[y*w] = gray[y*w]; out[y*w+w-1] = gray[y*w+w-1]; }
    return out;
  };

  /**
   * @private - Sobel edge detection.
   * Returns binary edge map (Uint8Array: 255=edge, 0=not).
   * Uses automatic threshold based on gradient statistics.
   */
  DocumentDetector.prototype._sobelEdges = function (gray, w, h) {
    var mag = new Uint16Array(w * h);
    var maxMag = 0;

    // Compute Sobel gradient magnitude
    for (var y = 1; y < h - 1; y++) {
      for (var x = 1; x < w - 1; x++) {
        var gx = -gray[(y-1)*w+(x-1)] - 2*gray[y*w+(x-1)] - gray[(y+1)*w+(x-1)]
                + gray[(y-1)*w+(x+1)] + 2*gray[y*w+(x+1)] + gray[(y+1)*w+(x+1)];
        var gy = -gray[(y-1)*w+(x-1)] - 2*gray[(y-1)*w+x] - gray[(y-1)*w+(x+1)]
                + gray[(y+1)*w+(x-1)] + 2*gray[(y+1)*w+x] + gray[(y+1)*w+(x+1)];
        // Fast magnitude approximation: |gx| + |gy| (avoids sqrt)
        var m = (gx < 0 ? -gx : gx) + (gy < 0 ? -gy : gy);
        mag[y * w + x] = m;
        if (m > maxMag) maxMag = m;
      }
    }

    // Automatic threshold: use Otsu's method on gradient histogram
    var threshold = this._otsuThreshold(mag, w, h, maxMag);
    // Ensure minimum threshold to filter noise and weak edges
    if (threshold < 60) threshold = 60;

    var edges = new Uint8Array(w * h);
    for (var i = 0; i < w * h; i++) {
      edges[i] = mag[i] >= threshold ? 255 : 0;
    }
    return edges;
  };

  /**
   * @private - Otsu's threshold on gradient magnitudes.
   * Finds the threshold that maximizes between-class variance.
   */
  DocumentDetector.prototype._otsuThreshold = function (mag, w, h, maxVal) {
    if (maxVal <= 0) return 0;
    var numBins = 256;
    var binScale = (numBins - 1) / maxVal;
    var hist = new Int32Array(numBins);

    for (var i = 0; i < w * h; i++) {
      var bin = Math.min(numBins - 1, (mag[i] * binScale) | 0);
      hist[bin]++;
    }

    var total = w * h;
    var sumAll = 0;
    for (var i = 0; i < numBins; i++) sumAll += i * hist[i];

    var wB = 0, wF = 0, sumB = 0;
    var maxVariance = 0, bestThresh = 0;

    for (var t = 0; t < numBins; t++) {
      wB += hist[t];
      if (wB === 0) continue;
      wF = total - wB;
      if (wF === 0) break;

      sumB += t * hist[t];
      var mB = sumB / wB;
      var mF = (sumAll - sumB) / wF;
      var diff = mB - mF;
      var variance = wB * wF * diff * diff;

      if (variance > maxVariance) {
        maxVariance = variance;
        bestThresh = t;
      }
    }

    return bestThresh / binScale;
  };

  /**
   * @private - Hough line transform.
   * Finds dominant straight lines in the edge image.
   * Returns array of {rho, theta} in image coordinates.
   */
  DocumentDetector.prototype._houghLines = function (edges, w, h) {
    var numAngles = this._numAngles;
    var sinT = this._sinTable;
    var cosT = this._cosTable;
    var diag = Math.ceil(Math.sqrt(w * w + h * h));
    var numRho = 2 * diag + 1;

    // Accumulator
    var acc = new Uint16Array(numAngles * numRho);

    // Vote: only iterate over edge pixels (sparse!)
    for (var y = 1; y < h - 1; y++) {
      for (var x = 1; x < w - 1; x++) {
        if (!edges[y * w + x]) continue;
        for (var ai = 0; ai < numAngles; ai++) {
          var rho = Math.round(x * cosT[ai] + y * sinT[ai]) + diag;
          acc[ai * numRho + rho]++;
        }
      }
    }

    // Find peaks: dynamic threshold based on expected document size
    // Expected card width = frame width * minWidthFraction (e.g., 40%)
    // minLineLen = 50% of expected card width
    var expectedCardWidth = w * this._minWidthFraction;
    var minLineLen = Math.max(20, expectedCardWidth * 0.5);
    var peaks = [];
    for (var ai = 0; ai < numAngles; ai++) {
      for (var ri = 0; ri < numRho; ri++) {
        var votes = acc[ai * numRho + ri];
        if (votes >= minLineLen) {
          peaks.push({ theta: ai * Math.PI / numAngles, rho: ri - diag, votes: votes });
        }
      }
    }

    // Sort by votes descending
    peaks.sort(function (a, b) { return b.votes - a.votes; });

    // Non-maximum suppression: merge nearby lines
    var lines = [];
    var rhoTolerance = Math.max(5, diag * 0.03);
    var thetaTolerance = 5 * Math.PI / 180; // 5 degrees

    for (var i = 0; i < peaks.length && lines.length < 40; i++) {
      var p = peaks[i];
      var duplicate = false;
      for (var j = 0; j < lines.length; j++) {
        var dTheta = Math.abs(p.theta - lines[j].theta);
        if (dTheta > Math.PI / 2) dTheta = Math.PI - dTheta; // wrap
        var dRho = Math.abs(p.rho - lines[j].rho);
        if (dTheta < thetaTolerance && dRho < rhoTolerance) {
          duplicate = true;
          break;
        }
      }
      if (!duplicate) {
        lines.push(p);
      }
    }

    return lines;
  };

  /**
   * @private - Find the best rectangle from detected Hough lines.
   *
   * Strategy:
   * 1. Classify lines as ~horizontal or ~vertical
   * 2. Find parallel line pairs (top/bottom, left/right)
   * 3. Compute 4 intersection points for each combo
   * 4. Validate: aspect ratio, area, angles
   * 5. Score by edge support (how many real edge pixels lie along each side)
   *    This prevents selecting rectangles that extend over hands/fingers
   */
  DocumentDetector.prototype._findRectangleFromLines = function (lines, pw, ph, scale, edges) {
    var self = this;
    var frameArea = pw * ph;
    var DEG = Math.PI / 180;

    // Classify lines into ~horizontal and ~vertical.
    // Boundary at exactly 45° / 135° with no gap so documents rotated up to
    // ±45° from landscape always have their edges classified into one bucket.
    var hLines = [];
    var vLines = [];

    for (var i = 0; i < lines.length; i++) {
      var angleDeg = lines[i].theta * 180 / Math.PI;
      if (angleDeg > 45 && angleDeg < 135) {
        hLines.push(lines[i]);
      } else {
        vLines.push(lines[i]);
      }
    }

    if (this.debug) console.log('[docuSnap] hLines:', hLines.length, 'vLines:', vLines.length);
    if (hLines.length < 2 || vLines.length < 2) { if (this.debug) console.warn('[docuSnap] KILLED: not enough h/v lines'); return null; }
    var _dbg = this.debug;
    var _rej = { hAngle:0, hVP:0, hDist:0, vAngle:0, vVP:0, vDist:0, corners:0, size:0, edgeRatio:0, aspect:0, docSize:0, convex:0, diag:0, angles:0, rotation:0, symmetry:0, edgeSupport:0 };

    // Limit search to top candidates
    if (hLines.length > 8) hLines = hLines.slice(0, 8);
    if (vLines.length > 8) vLines = vLines.slice(0, 8);

    // Known document aspect ratios: ID-1 credit card (1.586), ID-3 passport (1.417)
    var knownAspects = [1.586, 1.417];

    var bestResult = null;
    var bestScore = -1;

    for (var hi = 0; hi < hLines.length - 1; hi++) {
      for (var hj = hi + 1; hj < hLines.length; hj++) {
        var h1 = hLines[hi], h2 = hLines[hj];

        // Perspective-aware check: allow up to 20° for top/bottom edges.
        // When lines aren't parallel, validate their vanishing point is outside
        // the frame so convergence is physically plausible (not two random lines).
        var hAngleDiff = Math.abs(h1.theta - h2.theta);
        if (hAngleDiff > 20 * DEG) { _rej.hAngle++; continue; }
        if (hAngleDiff > 3 * DEG) {
          var hVP = self._lineIntersection(h1, h2);
          if (!hVP || (hVP.x > -0.5*pw && hVP.x < 1.5*pw && hVP.y > -0.5*ph && hVP.y < 1.5*ph)) { _rej.hVP++; continue; }
        }

        // Reasonable separation
        var hDist = Math.abs(h1.rho - h2.rho);
        if (hDist < ph * 0.1 || hDist > ph * 0.95) { _rej.hDist++; continue; }

        for (var vi = 0; vi < vLines.length - 1; vi++) {
          for (var vj = vi + 1; vj < vLines.length; vj++) {
            var v1 = vLines[vi], v2 = vLines[vj];

            // Perspective-aware check: left/right edges converge for any camera tilt.
            // topW/botW = 0.9 → vAngleDiff ≈ 11°; topW/botW = 0.7 → ≈ 28°.
            // Allow up to 25° and validate VP is outside the frame.
            var vAngleDiff = Math.abs(v1.theta - v2.theta);
            if (vAngleDiff > Math.PI / 2) vAngleDiff = Math.PI - vAngleDiff;
            if (vAngleDiff > 25 * DEG) { _rej.vAngle++; continue; }
            if (vAngleDiff > 3 * DEG) {
              var vVP = self._lineIntersection(v1, v2);
              if (!vVP || (vVP.x > -0.5*pw && vVP.x < 1.5*pw && vVP.y > -0.5*ph && vVP.y < 1.5*ph)) { _rej.vVP++; continue; }
            }

            // Reasonable separation
            var vDist = Math.abs(v1.rho - v2.rho);
            if (vDist < pw * 0.1 || vDist > pw * 0.95) { _rej.vDist++; continue; }

            // Compute 4 corners
            var corners = [
              self._lineIntersection(h1, v1),
              self._lineIntersection(h1, v2),
              self._lineIntersection(h2, v1),
              self._lineIntersection(h2, v2),
            ];

            // All corners must be in frame (allow 5% outside)
            var margin = -pw * 0.05;
            var allInFrame = true;
            for (var ci = 0; ci < 4; ci++) {
              if (!corners[ci] || corners[ci].x < margin || corners[ci].x > pw - margin ||
                  corners[ci].y < margin || corners[ci].y > ph - margin) {
                allInFrame = false;
                break;
              }
            }
            if (!allInFrame) { _rej.corners++; continue; }

            // Sort corners: TL, TR, BR, BL
            var sorted = self._sortCorners(corners);

            // Correct outlier corner using parallelogram geometry
            sorted = self._correctCornerParallelogram(sorted);

            // Side lengths
            var topW = Math.hypot(sorted[1].x - sorted[0].x, sorted[1].y - sorted[0].y);
            var botW = Math.hypot(sorted[2].x - sorted[3].x, sorted[2].y - sorted[3].y);
            var leftH = Math.hypot(sorted[3].x - sorted[0].x, sorted[3].y - sorted[0].y);
            var rightH = Math.hypot(sorted[2].x - sorted[1].x, sorted[2].y - sorted[1].y);

            var avgW = (topW + botW) / 2;
            var avgH = (leftH + rightH) / 2;
            if (avgW < 5 || avgH < 5) { _rej.size++; continue; }

            // Opposite sides similar
            var widthRatio = Math.min(topW, botW) / Math.max(topW, botW);
            var heightRatio = Math.min(leftH, rightH) / Math.max(leftH, rightH);
            if (widthRatio < 0.6 || heightRatio < 0.6) { _rej.edgeRatio++; continue; }

            // Aspect ratio - document must be landscape (wider than tall)
            // and match credit card (1.586) or passport (1.42) aspect ratios
            var aspect = avgW / avgH;  // < 1.0 for portrait = rejected
            if (aspect < self._minAspect || aspect > self._maxAspect) { _rej.aspect++; continue; }

            // Size checks
            var quadArea = self._quadArea(sorted);
            var areaFrac = quadArea / frameArea;
            var widthFrac = avgW / pw;  // Width-based doc size
            if (widthFrac < self._minWidthFraction || areaFrac > self._maxAreaFraction) { _rej.docSize++; continue; }

            // --- HOMOGRAPHY-BASED QUALITY CHECK ---
            // Instead of fragile angle-based patterns, use geometric quality metrics
            // that are stable under perspective distortion.
            
            // 1. Convexity check: quad must be convex (cross products same sign)
            var cross01 = (sorted[1].x - sorted[0].x) * (sorted[2].y - sorted[1].y) -
                          (sorted[1].y - sorted[0].y) * (sorted[2].x - sorted[1].x);
            var cross12 = (sorted[2].x - sorted[1].x) * (sorted[3].y - sorted[2].y) -
                          (sorted[2].y - sorted[1].y) * (sorted[3].x - sorted[2].x);
            var cross23 = (sorted[3].x - sorted[2].x) * (sorted[0].y - sorted[3].y) -
                          (sorted[3].y - sorted[2].y) * (sorted[0].x - sorted[3].x);
            var cross30 = (sorted[0].x - sorted[3].x) * (sorted[1].y - sorted[0].y) -
                          (sorted[0].y - sorted[3].y) * (sorted[1].x - sorted[0].x);
            var allPositive = cross01 > 0 && cross12 > 0 && cross23 > 0 && cross30 > 0;
            var allNegative = cross01 < 0 && cross12 < 0 && cross23 < 0 && cross30 < 0;
            if (!allPositive && !allNegative) { _rej.convex++; continue; }
            
            // 2. Diagonal ratio check: diagonals should be similar length (not too skewed)
            var diag1 = Math.hypot(sorted[2].x - sorted[0].x, sorted[2].y - sorted[0].y);  // TL to BR
            var diag2 = Math.hypot(sorted[3].x - sorted[1].x, sorted[3].y - sorted[1].y);  // TR to BL
            var diagRatio = Math.min(diag1, diag2) / Math.max(diag1, diag2);
            if (diagRatio < 0.5) { _rej.diag++; continue; }
            
            // 3. Corner angle sanity check (very loose - just reject extreme distortion)
            var minAngle = 45, maxAngle = 135;  // Much more permissive than before
            var anglesOk = true;
            for (var ai = 0; ai < 4; ai++) {
              var prev = sorted[(ai + 3) % 4];
              var curr = sorted[ai];
              var next = sorted[(ai + 1) % 4];
              var ax = prev.x - curr.x, ay = prev.y - curr.y;
              var bx = next.x - curr.x, by = next.y - curr.y;
              var dot = ax * bx + ay * by;
              var m1 = Math.sqrt(ax * ax + ay * ay);
              var m2 = Math.sqrt(bx * bx + by * by);
              if (m1 < 0.001 || m2 < 0.001) { anglesOk = false; break; }
              var cosA = dot / (m1 * m2);
              var aDeg = Math.acos(Math.max(-1, Math.min(1, cosA))) * 180 / Math.PI;
              if (aDeg < minAngle || aDeg > maxAngle) { anglesOk = false; break; }
            }
            if (!anglesOk) { _rej.angles++; continue; }

            // --- ROTATION CHECK ---
            // Document must be roughly horizontal (within +/- 30 degrees)
            var topDx = sorted[1].x - sorted[0].x;  // TR.x - TL.x
            var topDy = sorted[1].y - sorted[0].y;  // TR.y - TL.y
            var rotationRad = Math.atan2(topDy, topDx);
            var rotationDeg = Math.abs(rotationRad * 180 / Math.PI);
            // Allow +/- 45 degrees from horizontal (0°) or if document is upside down (180°)
            if (rotationDeg > 45 && rotationDeg < 135) { _rej.rotation++; continue; }

            // --- TRAPEZOID SYMMETRY CHECK ---
            // For perspective distortion of a flat document the midpoints of the top
            // and bottom edges must be roughly aligned horizontally (camera is roughly
            // above the document centre). A parallelogram-like asymmetry indicates
            // arbitrary skew that is NOT a perspective projection of a flat surface.
            var topMidX = (sorted[0].x + sorted[1].x) / 2;
            var botMidX = (sorted[3].x + sorted[2].x) / 2;
            var midXAlignFrac = Math.abs(topMidX - botMidX) / pw;
            if (midXAlignFrac > 0.20) { _rej.symmetry++; continue; }
            var symmetryScore = 1.0 - (midXAlignFrac / 0.20);

            // --- EDGE SUPPORT SCORING ---
            // Sample along each side of the rectangle and check what fraction
            // of sampled points have actual edge pixels nearby.
            // This penalizes rectangles whose sides pass through non-edge areas
            // (e.g., fingers/hands/background).
            var edgeSupport = self._measureEdgeSupport(sorted, edges, pw, ph);

            if (edgeSupport < 0.15) { _rej.edgeSupport++; continue; }

            // Rectangularity
            var rectArea = avgW * avgH;
            var rectangularity = Math.min(quadArea / rectArea, 1.0);

            // Aspect ratio closeness to nearest known document type
            // Score 1.0 when exact match to any known type, lower when further away
            var minAspectDist = Math.min.apply(null, knownAspects.map(function(a) { return Math.abs(aspect - a); }));
            var aspectScore = 1.0 - Math.min(minAspectDist / 0.2, 0.5);

            // --- CENTER BIAS ---
            // Prefer rectangles whose center is near the frame center
            // Users typically hold documents in the center of the frame
            var rectCenterX = (sorted[0].x + sorted[1].x + sorted[2].x + sorted[3].x) / 4;
            var rectCenterY = (sorted[0].y + sorted[1].y + sorted[2].y + sorted[3].y) / 4;
            var frameCenterX = pw / 2;
            var frameCenterY = ph / 2;
            var centerDist = Math.hypot(rectCenterX - frameCenterX, rectCenterY - frameCenterY);
            var maxDist = Math.hypot(pw / 2, ph / 2);  // Corner to center distance
            var centerScore = 1.0 - (centerDist / maxDist);  // 1.0 = perfectly centered

            // Final score — core weights as designed:
            // rectangularity and symmetry act as hard filters above; they don't dilute the score.
            var tightness = 1.0 - areaFrac;
            var score = edgeSupport * 0.40
                      + aspectScore * 0.30
                      + tightness   * 0.15
                      + centerScore * 0.15;

            if (score > bestScore) {
              bestScore = score;
              var invScale = 1 / scale;
              bestResult = {
                bbox: {
                  x: sorted[0].x * invScale,
                  y: sorted[0].y * invScale,
                  w: avgW * invScale,
                  h: avgH * invScale,
                },
                confidence: edgeSupport,
                classIndex: -1,
                cornerPoints: {
                  topLeftCorner:     { x: sorted[0].x * invScale, y: sorted[0].y * invScale },
                  topRightCorner:    { x: sorted[1].x * invScale, y: sorted[1].y * invScale },
                  bottomRightCorner: { x: sorted[2].x * invScale, y: sorted[2].y * invScale },
                  bottomLeftCorner:  { x: sorted[3].x * invScale, y: sorted[3].y * invScale },
                },
              };
            }
          }
        }
      }
    }

    if (_dbg) console.log('[docuSnap] rejection counts:', _rej, '| best score:', bestResult ? Math.round((bestResult.confidence||0)*100)+'%' : 'none');
    return bestResult;
  };

  /**
   * @private - Measure how well a candidate rectangle's edges are supported
   * by actual edge pixels in the edge map.
   *
   * Uses normalized support ratio: edge_pixels_near_line / line_length
   * Also penalizes gaps (consecutive samples without edges) to catch
   * false edges from hands/background that have discontinuities.
   *
   * @param {Array} corners - [TL, TR, BR, BL] sorted corners
   * @param {Uint8Array} edges - binary edge map
   * @param {number} w - edge map width
   * @param {number} h - edge map height
   * @returns {number} 0.0 to 1.0, support score with gap penalty
   */
  DocumentDetector.prototype._measureEdgeSupport = function (corners, edges, w, h) {
    var tolerance = 5;   // pixels — how far from the line to look for edges
    var minSupportRequired = 0.15;  // If any single side < 15%, return that (not avg)
    var maxGapFraction = 0.40;  // Allow gaps from rounded corners / fingers

    // 4 sides: TL→TR, TR→BR, BR→BL, BL→TL
    var sides = [
      [corners[0], corners[1]],
      [corners[1], corners[2]],
      [corners[2], corners[3]],
      [corners[3], corners[0]],
    ];

    var minSupport = 1.0;
    var totalSupport = 0;
    var maxGapPenalty = 0;

    for (var si = 0; si < 4; si++) {
      var p1 = sides[si][0], p2 = sides[si][1];
      var lineLen = Math.hypot(p2.x - p1.x, p2.y - p1.y);
      if (lineLen < 5) return 0;  // Too short

      // Sample along line at 1-pixel intervals
      var numSamples = Math.max(10, Math.floor(lineLen));
      var supported = 0;
      var currentGap = 0;
      var maxGap = 0;

      for (var s = 0; s <= numSamples; s++) {
        var t = s / numSamples;
        var px = Math.round(p1.x + t * (p2.x - p1.x));
        var py = Math.round(p1.y + t * (p2.y - p1.y));

        // Check if there's an edge pixel within tolerance radius
        var hasEdge = false;
        for (var dy = -tolerance; dy <= tolerance && !hasEdge; dy++) {
          for (var dx = -tolerance; dx <= tolerance && !hasEdge; dx++) {
            var nx = px + dx;
            var ny = py + dy;
            if (nx >= 0 && nx < w && ny >= 0 && ny < h) {
              if (edges[ny * w + nx]) hasEdge = true;
            }
          }
        }

        if (hasEdge) {
          supported++;
          currentGap = 0;
        } else {
          currentGap++;
          if (currentGap > maxGap) maxGap = currentGap;
        }
      }

      var sideSupport = supported / (numSamples + 1);
      minSupport = Math.min(minSupport, sideSupport);
      totalSupport += sideSupport;

      // Track worst gap across all sides (as fraction of side length)
      var gapFraction = maxGap / (numSamples + 1);
      if (gapFraction > maxGapPenalty) maxGapPenalty = gapFraction;
    }

    // Return average support, but penalize if any side is weak or has large gaps
    var avgSupport = totalSupport / 4;
    var baseScore = minSupport < minSupportRequired ? minSupport : avgSupport;
    
    // Apply mild gap penalty only for very large gaps
    // (Tightness preference handles false edges better than gap penalty)
    if (maxGapPenalty > maxGapFraction) {
      var gapPenalty = (maxGapPenalty - maxGapFraction) / (1.0 - maxGapFraction);
      baseScore *= (1.0 - gapPenalty * 0.25);  // Reduced: up to 25% penalty for severe gaps
    }
    
    return baseScore;
  };

  /**
   * @private - Compute intersection of two Hough lines.
   * Line i: x*cos(θi) + y*sin(θi) = ρi
   * Returns {x, y} or null if parallel.
   */
  DocumentDetector.prototype._lineIntersection = function (l1, l2) {
    var c1 = Math.cos(l1.theta), s1 = Math.sin(l1.theta);
    var c2 = Math.cos(l2.theta), s2 = Math.sin(l2.theta);
    var det = c1 * s2 - c2 * s1;
    if (Math.abs(det) < 1e-6) return null;
    return {
      x: (l1.rho * s2 - l2.rho * s1) / det,
      y: (l2.rho * c1 - l1.rho * c2) / det,
    };
  };

  /** @private - Sort 4 corners to TL, TR, BR, BL order */
  DocumentDetector.prototype._sortCorners = function (pts) {
    var cx = 0, cy = 0;
    for (var i = 0; i < pts.length; i++) { cx += pts[i].x; cy += pts[i].y; }
    cx /= pts.length;
    cy /= pts.length;

    var tl = null, tr = null, br = null, bl = null;
    for (var i = 0; i < pts.length; i++) {
      if (pts[i].x <= cx && pts[i].y <= cy) {
        if (!tl || (pts[i].x + pts[i].y) < (tl.x + tl.y)) tl = pts[i];
      }
      if (pts[i].x > cx && pts[i].y <= cy) {
        if (!tr || (pts[i].x - pts[i].y) > (tr.x - tr.y)) tr = pts[i];
      }
      if (pts[i].x > cx && pts[i].y > cy) {
        if (!br || (pts[i].x + pts[i].y) > (br.x + br.y)) br = pts[i];
      }
      if (pts[i].x <= cx && pts[i].y > cy) {
        if (!bl || (pts[i].y - pts[i].x) > (bl.y - bl.x)) bl = pts[i];
      }
    }

    // Fallback: sort clockwise by angle from centroid, then rotate so TL (min x+y) is first.
    // A plain sorted[0..3] assignment only works if the TL corner happens to have the
    // most-negative angle, which is not guaranteed for all document orientations.
    if (!tl || !tr || !br || !bl) {
      var angSorted = pts.slice().sort(function (a, b) {
        return Math.atan2(a.y - cy, a.x - cx) - Math.atan2(b.y - cy, b.x - cx);
      });
      // Find the index of the corner with the smallest x+y sum (closest to top-left)
      var tlIdx = 0;
      for (var fi = 1; fi < 4; fi++) {
        if (angSorted[fi].x + angSorted[fi].y < angSorted[tlIdx].x + angSorted[tlIdx].y) tlIdx = fi;
      }
      tl = angSorted[tlIdx];
      tr = angSorted[(tlIdx + 1) % 4];
      br = angSorted[(tlIdx + 2) % 4];
      bl = angSorted[(tlIdx + 3) % 4];
    }

    return [tl, tr, br, bl];
  };

  /**
   * @private - Correct outlier corner using parallelogram geometry.
   * For a perfect rectangle/parallelogram: TL + BR = TR + BL (diagonals bisect each other).
   * If 3 corners are good, the 4th can be computed from them.
   *
   * @param {Array} corners - [TL, TR, BR, BL] sorted corners
   * @returns {Array} - Corrected corners
   */
  DocumentDetector.prototype._correctCornerParallelogram = function (corners) {
    // Make a copy so we don't mutate the original
    var tl = { x: corners[0].x, y: corners[0].y };
    var tr = { x: corners[1].x, y: corners[1].y };
    var br = { x: corners[2].x, y: corners[2].y };
    var bl = { x: corners[3].x, y: corners[3].y };

    // For a parallelogram, opposite corners sum to the same value:
    // TL + BR = TR + BL  (the diagonals bisect each other)
    //
    // Given 3 corners, we can predict the 4th:
    // TL_pred = TR + BL - BR
    // TR_pred = TL + BR - BL
    // BR_pred = TR + BL - TL
    // BL_pred = TL + BR - TR
    var predictions = [
      { x: tr.x + bl.x - br.x, y: tr.y + bl.y - br.y },  // TL predicted
      { x: tl.x + br.x - bl.x, y: tl.y + br.y - bl.y },  // TR predicted
      { x: tr.x + bl.x - tl.x, y: tr.y + bl.y - tl.y },  // BR predicted
      { x: tl.x + br.x - tr.x, y: tl.y + br.y - tr.y },  // BL predicted
    ];

    var current = [tl, tr, br, bl];

    // Calculate error for each corner (distance from its predicted position)
    var errors = [];
    for (var i = 0; i < 4; i++) {
      errors.push({
        idx: i,
        error: Math.hypot(current[i].x - predictions[i].x, current[i].y - predictions[i].y)
      });
    }

    // Sort by error to find the outlier
    errors.sort(function(a, b) { return b.error - a.error; });
    var outlier = errors[0];
    var secondLargest = errors[1];

    // Only correct if:
    // 1. The largest error is very significant (> 15 pixels in processing coords)
    // 2. It's a clear outlier (> 5x the second largest error)
    // 3. Other corners are well-aligned (second error < 5px)
    // This prevents over-correction when detection is generally bad
    if (outlier.error > 15 && outlier.error > secondLargest.error * 5 && secondLargest.error < 5) {
      current[outlier.idx] = predictions[outlier.idx];
    }

    return current;
  };

  /** @private - Area of quadrilateral (Shoelace) */
  DocumentDetector.prototype._quadArea = function (pts) {
    var area = 0;
    for (var i = 0; i < pts.length; i++) {
      var j = (i + 1) % pts.length;
      area += pts[i].x * pts[j].y;
      area -= pts[j].x * pts[i].y;
    }
    return Math.abs(area) / 2;
  };

  // =========================================================================
  // docuSnap — main class
  // =========================================================================

  /**
   * Calculates distance between two points.
   */
  function distance(p1, p2) {
    return Math.hypot(p1.x - p2.x, p1.y - p2.y);
  }

  /**
   * Main docuSnap class — document detection and quality assessment.
   *
   * v2.1.0: Pure JS rectangle detector with aspect ratio filtering.
   * No ML model or external dependencies needed.
   *
   * @param {object} [options]
   * @param {number} [options.minAspectRatio=1.2] - min width/height for card detection
   * @param {number} [options.maxAspectRatio=1.8] - max width/height for card detection
   * @param {number} [options.minWidthFraction=0.40] - min document width as fraction of frame width
   * @param {string|ArrayBuffer} [options.modelUrl] - (deprecated, ignored)
   */
  class docuSnap {
    constructor(options) {
      options = options || {};
      this._quality = new QualityAssessor();
      this._detector = new DocumentDetector({
        minAspectRatio: options.minAspectRatio,
        maxAspectRatio: options.maxAspectRatio,
        minWidthFraction: options.minWidthFraction,
      });
      this._ready = false;
    }

    /**
     * Initializes the detector. Must be called before detect/highlight/assess.
     */
    async init() {
      await this._detector.init();
      this._ready = true;
    }

    /**
     * Returns true if the model has been loaded.
     */
    isReady() {
      return this._ready;
    }

    /**
     * Detects a document in the image and returns corner points + bounding box.
     *
     * @param {HTMLImageElement|HTMLCanvasElement|HTMLVideoElement} image
     * @returns {object|null} { bbox, confidence, classIndex, cornerPoints } or null
     */
    async detect(image) {
      var canvas = this._imageToCanvas(image);
      var ctx = canvas.getContext("2d");
      var imageData = ctx.getImageData(0, 0, canvas.width, canvas.height);
      return await this._detector.detect(imageData.data, canvas.width, canvas.height);
    }

    /**
     * Highlights the detected document in the image.
     * @param {HTMLImageElement|HTMLCanvasElement} image
     * @param {object} [options] - { color, thickness }
     * @returns {HTMLCanvasElement} canvas with highlighted document
     */
    async highlightPaper(image, options) {
      options = options || {};
      options.color = options.color || "orange";
      options.thickness = options.thickness || 10;

      var canvas = this._imageToCanvas(image);
      var ctx = canvas.getContext("2d");

      var imageData = ctx.getImageData(0, 0, canvas.width, canvas.height);
      var detection = await this._detector.detect(imageData.data, canvas.width, canvas.height);

      if (detection && detection.cornerPoints) {
        var cp = detection.cornerPoints;
        if (cp.topLeftCorner && cp.topRightCorner && cp.bottomLeftCorner && cp.bottomRightCorner) {
          ctx.strokeStyle = options.color;
          ctx.lineWidth = options.thickness;
          ctx.beginPath();
          ctx.moveTo(cp.topLeftCorner.x, cp.topLeftCorner.y);
          ctx.lineTo(cp.topRightCorner.x, cp.topRightCorner.y);
          ctx.lineTo(cp.bottomRightCorner.x, cp.bottomRightCorner.y);
          ctx.lineTo(cp.bottomLeftCorner.x, cp.bottomLeftCorner.y);
          ctx.closePath();
          ctx.stroke();
        }
      }

      return canvas;
    }

    /**
     * Extracts the detected document from the image with perspective correction.
     * Returns a straightened document with configurable margin.
     *
     * @param {HTMLImageElement|HTMLCanvasElement} image
     * @param {number} resultWidth - desired output width (document area, excluding margin)
     * @param {number} resultHeight - desired output height (document area, excluding margin)
     * @param {object} [cornerPoints] - optional pre-computed corner points
     * @param {object} [options] - { margin: 0.25 } margin as fraction of document size
     * @returns {HTMLCanvasElement|null} canvas with extracted document, or null
     */
    async extractPaper(image, resultWidth, resultHeight, cornerPoints, options) {
      options = options || {};
      var margin = options.margin !== undefined ? options.margin : 0.25;

      var srcCanvas = this._imageToCanvas(image);
      var srcCtx = srcCanvas.getContext("2d");

      if (!cornerPoints) {
        var imageData = srcCtx.getImageData(0, 0, srcCanvas.width, srcCanvas.height);
        var detection = await this._detector.detect(imageData.data, srcCanvas.width, srcCanvas.height);
        if (!detection) return null;
        cornerPoints = detection.cornerPoints;
      }

      var cp = cornerPoints;
      if (!cp.topLeftCorner || !cp.topRightCorner || !cp.bottomLeftCorner || !cp.bottomRightCorner) {
        return null;
      }

      // Source corners (detected quadrilateral)
      var srcPts = [
        [cp.topLeftCorner.x, cp.topLeftCorner.y],
        [cp.topRightCorner.x, cp.topRightCorner.y],
        [cp.bottomRightCorner.x, cp.bottomRightCorner.y],
        [cp.bottomLeftCorner.x, cp.bottomLeftCorner.y],
      ];

      // Calculate margin in pixels
      var marginX = Math.round(resultWidth * margin);
      var marginY = Math.round(resultHeight * margin);

      // Total output size including margin
      var totalWidth = resultWidth + 2 * marginX;
      var totalHeight = resultHeight + 2 * marginY;

      // Destination corners (rectified rectangle with margin offset)
      var dstPts = [
        [marginX, marginY],                           // TL
        [marginX + resultWidth, marginY],             // TR
        [marginX + resultWidth, marginY + resultHeight], // BR
        [marginX, marginY + resultHeight],            // BL
      ];

      // Compute inverse homography (dst -> src) for backward mapping
      var H = this._computeHomography(dstPts, srcPts);
      if (!H) return null;

      // Get source image data
      var srcData = srcCtx.getImageData(0, 0, srcCanvas.width, srcCanvas.height);
      var srcW = srcCanvas.width;
      var srcH = srcCanvas.height;

      // Create output canvas
      var outCanvas = document.createElement("canvas");
      outCanvas.width = totalWidth;
      outCanvas.height = totalHeight;
      var outCtx = outCanvas.getContext("2d");
      var outData = outCtx.createImageData(totalWidth, totalHeight);

      // Warp using inverse mapping with bilinear interpolation
      for (var y = 0; y < totalHeight; y++) {
        for (var x = 0; x < totalWidth; x++) {
          // Apply homography: src = H * dst
          var w = H[6] * x + H[7] * y + H[8];
          var srcX = (H[0] * x + H[1] * y + H[2]) / w;
          var srcY = (H[3] * x + H[4] * y + H[5]) / w;

          // Bilinear interpolation
          var outIdx = (y * totalWidth + x) * 4;

          if (srcX >= 0 && srcX < srcW - 1 && srcY >= 0 && srcY < srcH - 1) {
            var x0 = Math.floor(srcX), y0 = Math.floor(srcY);
            var fx = srcX - x0, fy = srcY - y0;

            for (var c = 0; c < 4; c++) {
              var i00 = (y0 * srcW + x0) * 4 + c;
              var i10 = (y0 * srcW + x0 + 1) * 4 + c;
              var i01 = ((y0 + 1) * srcW + x0) * 4 + c;
              var i11 = ((y0 + 1) * srcW + x0 + 1) * 4 + c;

              var v = srcData.data[i00] * (1 - fx) * (1 - fy) +
                      srcData.data[i10] * fx * (1 - fy) +
                      srcData.data[i01] * (1 - fx) * fy +
                      srcData.data[i11] * fx * fy;

              outData.data[outIdx + c] = Math.round(v);
            }
          } else {
            // Outside source bounds - white background
            outData.data[outIdx] = 255;
            outData.data[outIdx + 1] = 255;
            outData.data[outIdx + 2] = 255;
            outData.data[outIdx + 3] = 255;
          }
        }
      }

      outCtx.putImageData(outData, 0, 0);
      return outCanvas;
    }

    /**
     * Compute 3x3 homography matrix from 4 point correspondences.
     * Uses DLT (Direct Linear Transform) algorithm.
     * @private
     */
    _computeHomography(srcPts, dstPts) {
      // Build 8x9 matrix A for homogeneous system Ah = 0
      var A = [];
      for (var i = 0; i < 4; i++) {
        var sx = srcPts[i][0], sy = srcPts[i][1];
        var dx = dstPts[i][0], dy = dstPts[i][1];
        A.push([-sx, -sy, -1, 0, 0, 0, sx * dx, sy * dx, dx]);
        A.push([0, 0, 0, -sx, -sy, -1, sx * dy, sy * dy, dy]);
      }

      // Solve using SVD (simplified for 4-point case)
      // We use a direct solve since we have exactly 8 equations for 8 unknowns (h9=1)
      var At = this._transpose(A);
      var AtA = this._matMul(At, A);
      
      // Solve AtA * h = 0 by finding eigenvector of smallest eigenvalue
      // For simplicity, use power iteration on inverse
      var h = this._solveHomogeneous(AtA);
      if (!h) return null;

      return h;
    }

    /** @private - Transpose matrix */
    _transpose(M) {
      var rows = M.length, cols = M[0].length;
      var T = [];
      for (var j = 0; j < cols; j++) {
        T[j] = [];
        for (var i = 0; i < rows; i++) {
          T[j][i] = M[i][j];
        }
      }
      return T;
    }

    /** @private - Matrix multiply */
    _matMul(A, B) {
      var rowsA = A.length, colsA = A[0].length;
      var colsB = B[0].length;
      var C = [];
      for (var i = 0; i < rowsA; i++) {
        C[i] = [];
        for (var j = 0; j < colsB; j++) {
          var sum = 0;
          for (var k = 0; k < colsA; k++) {
            sum += A[i][k] * B[k][j];
          }
          C[i][j] = sum;
        }
      }
      return C;
    }

    /** @private - Solve Ah=0 using inverse iteration */
    _solveHomogeneous(AtA) {
      var n = AtA.length;
      // Add small regularization and solve (AtA + εI)x = b
      // Use power iteration to find smallest eigenvector
      var h = [];
      for (var i = 0; i < n; i++) h[i] = 1;

      // Inverse iteration (find eigenvector of smallest eigenvalue)
      for (var iter = 0; iter < 100; iter++) {
        // Solve AtA * h_new = h using Gaussian elimination
        var h_new = this._gaussSolve(AtA, h);
        if (!h_new) return null;

        // Normalize
        var norm = 0;
        for (var i = 0; i < n; i++) norm += h_new[i] * h_new[i];
        norm = Math.sqrt(norm);
        if (norm < 1e-10) return null;
        for (var i = 0; i < n; i++) h_new[i] /= norm;

        h = h_new;
      }

      // Normalize so h[8] = 1
      if (Math.abs(h[8]) < 1e-10) return null;
      for (var i = 0; i < 9; i++) h[i] /= h[8];

      return h;
    }

    /** @private - Gaussian elimination solver */
    _gaussSolve(A, b) {
      var n = A.length;
      // Create augmented matrix
      var M = [];
      for (var i = 0; i < n; i++) {
        M[i] = A[i].slice();
        M[i].push(b[i]);
      }

      // Forward elimination with partial pivoting
      for (var col = 0; col < n; col++) {
        // Find pivot
        var maxRow = col;
        for (var row = col + 1; row < n; row++) {
          if (Math.abs(M[row][col]) > Math.abs(M[maxRow][col])) {
            maxRow = row;
          }
        }
        var tmp = M[col]; M[col] = M[maxRow]; M[maxRow] = tmp;

        if (Math.abs(M[col][col]) < 1e-10) {
          // Add regularization
          M[col][col] += 1e-6;
        }

        // Eliminate
        for (var row = col + 1; row < n; row++) {
          var f = M[row][col] / M[col][col];
          for (var j = col; j <= n; j++) {
            M[row][j] -= f * M[col][j];
          }
        }
      }

      // Back substitution
      var x = new Array(n);
      for (var i = n - 1; i >= 0; i--) {
        x[i] = M[i][n];
        for (var j = i + 1; j < n; j++) {
          x[i] -= M[i][j] * x[j];
        }
        x[i] /= M[i][i];
      }

      return x;
    }

    // =========================================================================
    // Quality Assessment Methods (pure JS — no OpenCV)
    // =========================================================================

    /**
     * Measures image sharpness using Laplacian variance.
     * @param {Uint8ClampedArray} rgba - RGBA pixel data
     * @param {number} w - width
     * @param {number} h - height
     * @returns {number} sharpness score
     */
    measureSharpness(rgba, w, h) {
      var gray = this._quality.rgbaToGrayscale(rgba, w, h);
      return this._quality.measureSharpness(gray, w, h);
    }

    /**
     * Measures brightness of the image.
     * @param {Uint8ClampedArray} rgba
     * @param {number} w
     * @param {number} h
     * @returns {number} mean brightness (0-255)
     */
    measureBrightness(rgba, w, h) {
      var gray = this._quality.rgbaToGrayscale(rgba, w, h);
      return this._quality.measureBrightness(gray, w, h);
    }

    /**
     * Detects glare/hotspots in the image.
     * @param {Uint8ClampedArray} rgba
     * @param {number} w
     * @param {number} h
     * @param {number} [threshold=240]
     * @returns {number} fraction of glare pixels (0.0 - 1.0)
     */
    detectGlare(rgba, w, h, threshold) {
      var gray = this._quality.rgbaToGrayscale(rgba, w, h);
      return this._quality.detectGlare(gray, w, h, threshold);
    }

    /**
     * Checks if all 4 document corners are detected and within frame margins.
     * (Pure JS — unchanged from v1)
     */
    checkDocumentCompleteness(cornerPoints, frameWidth, frameHeight, marginPx) {
      marginPx = marginPx !== undefined ? marginPx : 10;

      var tl = cornerPoints.topLeftCorner;
      var tr = cornerPoints.topRightCorner;
      var bl = cornerPoints.bottomLeftCorner;
      var br = cornerPoints.bottomRightCorner;
      var allCornersFound = !!(tl && tr && bl && br);

      if (!allCornersFound) {
        return { allCornersFound: false, allWithinMargin: false, details: {} };
      }

      function withinMargin(point) {
        return (
          point.x >= marginPx &&
          point.y >= marginPx &&
          point.x <= frameWidth - marginPx &&
          point.y <= frameHeight - marginPx
        );
      }

      var details = {
        topLeft: withinMargin(tl),
        topRight: withinMargin(tr),
        bottomLeft: withinMargin(bl),
        bottomRight: withinMargin(br),
      };

      var allWithinMargin = details.topLeft && details.topRight && details.bottomLeft && details.bottomRight;
      return { allCornersFound: true, allWithinMargin: allWithinMargin, details: details };
    }

    /**
     * Checks the document's size relative to the frame.
     * Returns width coverage (0-1) since aspect ratio is consistent.
     */
    measureDocumentSize(cornerPoints, frameWidth, frameHeight) {
      var tl = cornerPoints.topLeftCorner;
      var tr = cornerPoints.topRightCorner;
      var bl = cornerPoints.bottomLeftCorner;
      var br = cornerPoints.bottomRightCorner;

      if (!tl || !tr || !bl || !br) return 0;

      // Get bounding box width from corners
      var allX = [tl.x, tr.x, bl.x, br.x];
      var minX = Math.min.apply(null, allX);
      var maxX = Math.max.apply(null, allX);
      var bboxWidth = maxX - minX;

      // Return width coverage as fraction of frame width
      return bboxWidth / frameWidth;
    }

    /**
     * Performs a full quality assessment.
     *
     * @param {Uint8ClampedArray} rgba - RGBA pixel data
     * @param {number} w - width
     * @param {number} h - height
     * @param {object|null} cornerPoints
     * @param {object} [thresholds]
     * @returns {object} quality report
     */
    assessQuality(rgba, w, h, cornerPoints, thresholds) {
      thresholds = thresholds || {};
      var sharpnessMin = thresholds.sharpnessMin !== undefined ? thresholds.sharpnessMin : 100;
      var brightnessMin = thresholds.brightnessMin !== undefined ? thresholds.brightnessMin : 40;
      var glareMax = thresholds.glareMax !== undefined ? thresholds.glareMax : 0.10;
      // Pixel brightness threshold for glare detection (0-255).
      // 248 = only catch near-white hotspots; white card background (~230-247) is not flagged.
      var glareThreshold = thresholds.glareThreshold !== undefined ? thresholds.glareThreshold : 248;
      var documentSizeMin = thresholds.documentSizeMin !== undefined ? thresholds.documentSizeMin : 0.15;
      var cornerMarginPx = thresholds.cornerMarginPx !== undefined ? thresholds.cornerMarginPx : 10;

      // Convert to grayscale once, reuse for all quality checks
      var gray = this._quality.rgbaToGrayscale(rgba, w, h);

      var sharpness = this._quality.measureSharpness(gray, w, h);
      var brightness = this._quality.measureBrightness(gray, w, h);
      // Measure glare within document bounds for accuracy (falls back to whole frame if no corners)
      var glare = this._quality.detectGlareInRegion(gray, w, h, cornerPoints, glareThreshold);

      var completeness = { allCornersFound: false, allWithinMargin: false, details: {} };
      var documentSize = 0;

      if (cornerPoints) {
        completeness = this.checkDocumentCompleteness(cornerPoints, w, h, cornerMarginPx);
        documentSize = this.measureDocumentSize(cornerPoints, w, h);
      }

      var checks = {
        sharpness: { value: sharpness, pass: sharpness >= sharpnessMin },
        brightness: { value: brightness, pass: brightness >= brightnessMin, min: brightnessMin },
        glare: { value: glare, pass: glare <= glareMax },
        cornersFound: { value: completeness.allCornersFound, pass: completeness.allCornersFound },
        cornersWithinMargin: { value: completeness.allWithinMargin, pass: completeness.allWithinMargin },
        documentSize: { value: documentSize, pass: documentSize >= documentSizeMin },
      };

      var allPassed = true;
      var keys = Object.keys(checks);
      for (var i = 0; i < keys.length; i++) {
        if (!checks[keys[i]].pass) { allPassed = false; break; }
      }

      return { checks: checks, allPassed: allPassed };
    }

    /** @private - Convert image/video/canvas to canvas */
    _imageToCanvas(image) {
      var canvas = document.createElement("canvas");
      if (image instanceof HTMLVideoElement) {
        canvas.width = image.videoWidth;
        canvas.height = image.videoHeight;
      } else {
        canvas.width = image.naturalWidth || image.width;
        canvas.height = image.naturalHeight || image.height;
      }
      var ctx = canvas.getContext("2d");
      ctx.drawImage(image, 0, 0);
      return canvas;
    }
  }

  // ===========================================================================
  // DocumentAutoCapture - camera loop with quality gate & best-frame selection
  // ===========================================================================

  var State = {
    IDLE: "idle",
    DETECTING: "detecting",
    STAY_STILL: "stay_still",
    CAPTURED: "captured",
  };

  /**
   * DocumentAutoCapture orchestrates live camera capture with automatic
   * quality-gated document photo acquisition using rectangle detection.
   *
   * @param {object} options
   * @param {docuSnap} options.scanner - an initialized docuSnap instance
   * @param {HTMLVideoElement} options.videoElement
   * @param {HTMLCanvasElement} [options.canvasElement]
   * @param {object} [options.thresholds]
   * @param {number} [options.consecutiveFramesNeeded=5]
   * @param {number} [options.stayStillDurationMs=1000]
   * @param {number} [options.frameIntervalMs=100]
   * @param {function} options.onCapture
   * @param {function} [options.onStateChange]
   * @param {function} [options.onQualityReport]
   */
  class DocumentAutoCapture {
    constructor(options) {
      this._scanner = options.scanner;
      this._video = options.videoElement;
      this._canvas = options.canvasElement || null;
      this._thresholds = options.thresholds || {};
      this._consecutiveNeeded = options.consecutiveFramesNeeded || 5;
      this._stayStillMs = options.stayStillDurationMs || 1000;
      this._frameIntervalMs = options.frameIntervalMs || 100;
      this._onCapture = options.onCapture;
      this._onStateChange = options.onStateChange || function () { };
      this._onQualityReport = options.onQualityReport || function () { };
      this._onFrameData = options.onFrameData || null;  // Raw frame hook for DocuSnap API layer
      this._manualMode = options.manualMode || false;   // If true, never auto-fire capture

      this._state = State.IDLE;
      this._consecutiveGoodFrames = 0;
      this._candidates = [];
      this._stayStillStart = 0;
      this._animFrameId = null;
      this._lastEvalTime = 0;
      this._evaluating = false;

      // Bounding box smoothing state
      this._stableCorners = null;        // Ground truth corners (validated)
      this._displayCorners = null;       // Currently displayed corners (smoothed)
      this._cornerRejectCount = 0;       // Frames rejecting stable corners
      this._confirmFramesNeeded = 20;    // ~2s of missed frames before clearing the box
      
      // Kalman filter state for each corner (x, y independently)
      // State: [position, velocity], we track 8 values (4 corners x 2 coords)
      this._kalmanFilters = null;        // Array of 8 KalmanFilter1D instances
      
      // Bounding box overlay opacity state (0 = clear, 70 = max overlay)
      this._currentTransparency = 70;    // Start with overlay (will fade as quality improves)
      this._transparencyStep = 5;        // Change by 5% per frame
    }

    start() {
      this._state = State.DETECTING;
      this._consecutiveGoodFrames = 0;
      this._candidates = [];
      this._evaluating = false;
      this._stableCorners = null;
      this._displayCorners = null;
      this._cornerRejectCount = 0;
      this._kalmanFilters = null;
      this._currentTransparency = 70;    // Reset overlay opacity
      // Persistent small canvas reused every frame for detection (avoids per-frame allocation)
      if (!this._detCanvas) this._detCanvas = document.createElement("canvas");
      this._onStateChange(State.DETECTING, "Position document in frame");
      this._tick();
    }

    stop() {
      this._state = State.IDLE;
      if (this._animFrameId) {
        cancelAnimationFrame(this._animFrameId);
        this._animFrameId = null;
      }
    }

    reset() {
      this.stop();
      this.start();
    }

    /**
     * Manually trigger a capture immediately.
     * Works in both auto-detect and manual modes.
     * In manual mode (manualMode:true) this is the only way to fire a capture.
     */
    capture() {
      if (this._state === State.IDLE || this._state === State.CAPTURED) return;
      // Ensure at least one candidate exists (use stable corners from Kalman smoother)
      if (this._candidates.length === 0) {
        this._candidates.push({
          sharpness:   0,
          fullCorners: this._stableCorners || null,
          report:      null,
        });
      }
      this._selectBestFrame();
    }

    getState() {
      return this._state;
    }

    /** @private */
    _tick() {
      var self = this;
      this._animFrameId = requestAnimationFrame(function (timestamp) {
        if (self._state === State.IDLE || self._state === State.CAPTURED) return;

        // Skip if still processing previous frame (async inference)
        if (self._evaluating) {
          self._tick();
          return;
        }

        if (timestamp - self._lastEvalTime >= self._frameIntervalMs) {
          self._lastEvalTime = timestamp;
          self._evaluating = true;
          self._evaluateFrame().then(function () {
            self._evaluating = false;
          }).catch(function (err) {
            console.error("docuSnap: frame evaluation error:", err);
            self._evaluating = false;
          });
        }

        self._tick();
      });
    }

    /**
     * @private
     *
     * Two-tier canvas pipeline for good mobile frame rates:
     *
     * ┌─────────────────────────────────────────────────────────────────────┐
     * │ DETECTION  (this._detCanvas, max 640px longest side)               │
     * │   – created once in start(), reused every frame                    │
     * │   – getImageData on ~0.6 MB instead of 8 MB for 1080p             │
     * │   – Hough + quality assessment on 640×360 ≈ 8× faster than 1080p │
     * ├─────────────────────────────────────────────────────────────────────┤
     * │ DISPLAY    (this._canvas, max 720px longest side)                  │
     * │   – video drawn directly, never getImageData'd                     │
     * │   – overlay corners scaled up from detection space                 │
     * │   – dimensions set once on first valid frame, not every frame     │
     * ├─────────────────────────────────────────────────────────────────────┤
     * │ CAPTURE    (fresh full-res grab at _selectBestFrame time)          │
     * │   – corners scaled back to full video resolution for extraction    │
     * │   – high-quality JPEG from actual camera pixel count              │
     * └─────────────────────────────────────────────────────────────────────┘
     */
    async _evaluateFrame() {
      var video = this._video;
      if (video.readyState < video.HAVE_CURRENT_DATA) return;

      var vw = video.videoWidth;
      var vh = video.videoHeight;
      if (!vw || !vh) return;

      // ── Detection canvas (persistent, 640px max) ───────────────────────
      var DET_MAX = 640;
      var detScale = Math.min(1, DET_MAX / Math.max(vw, vh));
      var detW = Math.round(vw * detScale);
      var detH = Math.round(vh * detScale);

      var detCanvas = this._detCanvas;
      if (detCanvas.width !== detW || detCanvas.height !== detH) {
        detCanvas.width  = detW;
        detCanvas.height = detH;
      }
      var detCtx = detCanvas.getContext("2d");
      detCtx.drawImage(video, 0, 0, detW, detH);
      var imageData = detCtx.getImageData(0, 0, detW, detH);

      // ── Detect + assess at detection resolution ────────────────────────
      var detection  = this._scanner._detector.detect(imageData.data, detW, detH);
      var detCorners = detection ? detection.cornerPoints : null;

      var report = this._scanner.assessQuality(
        imageData.data, detW, detH, detCorners, this._thresholds
      );
      this._onQualityReport(report);

      // ── Scale corners to display space and full-video space ────────────
      // Display canvas is capped at 720px; full-video space used for capture.
      var DISP_MAX  = 720;
      var dispScale = Math.min(1, DISP_MAX / Math.max(vw, vh));
      var dispW     = Math.round(vw * dispScale);
      var dispH     = Math.round(vh * dispScale);

      var dispCorners = null;
      var fullCorners = null;
      if (detCorners) {
        var toDisp = dispW / detW;                      // det-space → display-space
        var toFull = detScale < 1 ? 1 / detScale : 1;  // det-space → full-video-space
        dispCorners = {
          topLeftCorner:     { x: detCorners.topLeftCorner.x     * toDisp, y: detCorners.topLeftCorner.y     * toDisp },
          topRightCorner:    { x: detCorners.topRightCorner.x    * toDisp, y: detCorners.topRightCorner.y    * toDisp },
          bottomRightCorner: { x: detCorners.bottomRightCorner.x * toDisp, y: detCorners.bottomRightCorner.y * toDisp },
          bottomLeftCorner:  { x: detCorners.bottomLeftCorner.x  * toDisp, y: detCorners.bottomLeftCorner.y  * toDisp },
        };
        fullCorners = {
          topLeftCorner:     { x: detCorners.topLeftCorner.x     * toFull, y: detCorners.topLeftCorner.y     * toFull },
          topRightCorner:    { x: detCorners.topRightCorner.x    * toFull, y: detCorners.topRightCorner.y    * toFull },
          bottomRightCorner: { x: detCorners.bottomRightCorner.x * toFull, y: detCorners.bottomRightCorner.y * toFull },
          bottomLeftCorner:  { x: detCorners.bottomLeftCorner.x  * toFull, y: detCorners.bottomLeftCorner.y  * toFull },
        };
      }

      // ── Kalman smoother operates in display space ──────────────────────
      this._updateStableCorners(dispCorners, dispW, dispH);

      // ── Draw to display canvas ─────────────────────────────────────────
      if (this._canvas) {
        // Resize only when dimensions actually change (avoids clearing + reflow every frame)
        if (this._canvas.width !== dispW || this._canvas.height !== dispH) {
          this._canvas.width  = dispW;
          this._canvas.height = dispH;
          this._canvas.style.aspectRatio = dispW + " / " + dispH;
        }
        var canvasCtx = this._canvas.getContext("2d");

        // Draw video at display resolution directly (no intermediate canvas copy)
        canvasCtx.drawImage(video, 0, 0, dispW, dispH);

        // Convexity guard: bowtie corners from Kalman crossing → reset
        if (this._displayCorners && !this._isCornersConvex(this._displayCorners)) {
          this._displayCorners = null;
          this._stableCorners  = null;
          this._kalmanFilters  = null;
        }

        if (this._displayCorners) {
          this._drawSpotlightOverlay(canvasCtx, this._displayCorners, report, dispW, dispH);
        } else {
          canvasCtx.fillStyle = "rgba(0, 0, 0, 0.5)";
          canvasCtx.fillRect(0, 0, dispW, dispH);
        }
      }

      // ── State machine ──────────────────────────────────────────────────
      if (this._state === State.DETECTING) {
        if (report.allPassed) {
          this._consecutiveGoodFrames++;
          if (this._consecutiveGoodFrames >= this._consecutiveNeeded) {
            this._state = State.STAY_STILL;
            this._stayStillStart = performance.now();
            this._candidates = [];
            this._onStateChange(State.STAY_STILL, "Hold still...");
          }
        } else {
          this._consecutiveGoodFrames = 0;
          this._emitInstruction(report);
        }
      }

      if (this._state === State.STAY_STILL) {
        if (!report.allPassed) {
          this._state = State.DETECTING;
          this._consecutiveGoodFrames = 0;
          this._candidates = [];
          this._onStateChange(State.DETECTING, "Position document in frame");
        } else {
          // Store detection-space corners scaled to full video resolution — the actual
          // high-quality frame is grabbed fresh from the video in _selectBestFrame.
          this._candidates.push({
            sharpness:   report.checks.sharpness.value,
            fullCorners: fullCorners,
            report:      report,
          });

          if (!this._manualMode && performance.now() - this._stayStillStart >= this._stayStillMs) {
            this._selectBestFrame();
          }
        }
      }

      // ── Raw frame data callback (DocuSnap public API layer) ───────────
      // Fired after state machine so `state` reflects any transition this tick.
      if (this._onFrameData) {
        this._onFrameData({
          detCanvas:   detCanvas,
          detCorners:  detCorners,
          dispCorners: dispCorners,
          fullCorners: fullCorners,
          state:       this._state,
          report:      report,
        });
      }
    }

    /** @private */
    async _selectBestFrame() {
      this._state = State.CAPTURED;

      // Pick sharpest candidate
      var best = this._candidates[0];
      for (var i = 1; i < this._candidates.length; i++) {
        if (this._candidates[i].sharpness > best.sharpness) best = this._candidates[i];
      }

      var video = this._video;
      var vw    = video.videoWidth;
      var vh    = video.videoHeight;

      // ── Grab one full-resolution frame from the live video ────────────
      // (candidates no longer store a JPEG per frame — saves memory and per-frame cost)
      var captureCanvas = document.createElement("canvas");
      captureCanvas.width  = vw;
      captureCanvas.height = vh;
      captureCanvas.getContext("2d").drawImage(video, 0, 0);
      var capturedImageData = captureCanvas.toDataURL("image/jpeg", 0.95);
      captureCanvas = null; // allow GC

      // ── Perspective-correct extraction ────────────────────────────────
      var self = this;
      var extractedDataUrl = null;

      try {
        var img = new Image();
        await new Promise(function (resolve, reject) {
          img.onload = resolve;
          img.onerror = reject;
          img.src = capturedImageData;
        });

        var cp = best.fullCorners;
        if (cp) {
          var _topW  = Math.hypot(cp.topRightCorner.x  - cp.topLeftCorner.x,  cp.topRightCorner.y  - cp.topLeftCorner.y);
          var _botW  = Math.hypot(cp.bottomRightCorner.x - cp.bottomLeftCorner.x, cp.bottomRightCorner.y - cp.bottomLeftCorner.y);
          var _lefH  = Math.hypot(cp.bottomLeftCorner.x  - cp.topLeftCorner.x,  cp.bottomLeftCorner.y  - cp.topLeftCorner.y);
          var _rigH  = Math.hypot(cp.bottomRightCorner.x - cp.topRightCorner.x, cp.bottomRightCorner.y - cp.topRightCorner.y);
          var _avgW  = (_topW + _botW) / 2;
          var _avgH  = (_lefH + _rigH) / 2;
          var _longSide = 856;
          var docWidth  = _avgW >= _avgH ? _longSide : Math.round(_longSide * _avgW / _avgH);
          var docHeight = _avgW >= _avgH ? Math.round(_longSide * _avgH / _avgW) : _longSide;
          var extracted = await self._scanner.extractPaper(img, docWidth, docHeight, cp, { margin: 0.25 });
          if (extracted) extractedDataUrl = extracted.toDataURL("image/jpeg", 0.95);
        }
      } catch (e) {
        console.error("docuSnap: extraction error:", e);
      }

      this._onStateChange(State.CAPTURED, "Document captured");
      this._onCapture({
        imageData:     capturedImageData,   // Full-resolution grab from video
        extractedData: extractedDataUrl,    // Perspective-corrected + 25% margin
        cornerPoints:  best.fullCorners,
        qualityReport: best.report,
      });
    }

    /** @private */
    _emitInstruction(report) {
      var checks = report.checks;
      var instruction = "Position document in frame";

      if (!checks.cornersFound.pass) {
        instruction = "Document not detected";
      } else if (!checks.cornersWithinMargin.pass) {
        instruction = "Move document away from edges";
      } else if (!checks.documentSize.pass) {
        instruction = "Move closer to document";
      } else if (!checks.sharpness.pass) {
        instruction = "Hold steady - image is blurry";
      } else if (!checks.glare.pass) {
        instruction = "Reduce glare - tilt document slightly";
      } else if (!checks.brightness.pass) {
        instruction = "Too dark - improve lighting";
      }

      this._onStateChange(State.DETECTING, instruction);
    }

    /** @private - Initialize Kalman filters for corner smoothing */
    _initKalmanFilters(corners) {
      var cornerKeys = ['topLeftCorner', 'topRightCorner', 'bottomLeftCorner', 'bottomRightCorner'];
      this._kalmanFilters = [];
      
      for (var i = 0; i < cornerKeys.length; i++) {
        var corner = corners[cornerKeys[i]];
        // Create filter for X coordinate
        this._kalmanFilters.push(new KalmanFilter1D(corner.x));
        // Create filter for Y coordinate  
        this._kalmanFilters.push(new KalmanFilter1D(corner.y));
      }
    }

    /** @private - Update stable corners with Kalman filter smoothing */
    _updateStableCorners(newCorners, frameW, frameH) {
      // No detection this frame
      if (!newCorners) {
        this._cornerRejectCount++;

        // While we still have filters, coast (predict-only) so the box drifts
        // smoothly on last known velocity rather than freezing or jumping.
        // For a stationary document velocity ≈ 0 so the box stays put.
        if (this._kalmanFilters && this._displayCorners) {
          var cornerKeys = ['topLeftCorner', 'topRightCorner', 'bottomLeftCorner', 'bottomRightCorner'];
          var coasted = {};
          for (var i = 0; i < cornerKeys.length; i++) {
            coasted[cornerKeys[i]] = {
              x: this._kalmanFilters[i * 2].coast(),
              y: this._kalmanFilters[i * 2 + 1].coast(),
            };
          }
          this._displayCorners = coasted;
        }

        // Only clear after ~2s of continuous missed frames
        if (this._cornerRejectCount >= this._confirmFramesNeeded) {
          this._stableCorners = null;
          this._displayCorners = null;
          this._kalmanFilters = null;
        }
        return;
      }

      // Reset reject count on any detection
      this._cornerRejectCount = 0;

      // First detection - initialize Kalman filters
      if (!this._kalmanFilters) {
        this._initKalmanFilters(newCorners);
        this._stableCorners = newCorners;
        this._displayCorners = newCorners;
        return;
      }

      // Update Kalman filters with new measurements
      var cornerKeys = ['topLeftCorner', 'topRightCorner', 'bottomLeftCorner', 'bottomRightCorner'];
      var smoothed = {};
      
      for (var i = 0; i < cornerKeys.length; i++) {
        var key = cornerKeys[i];
        var measurement = newCorners[key];
        
        // Update X and Y filters
        var filteredX = this._kalmanFilters[i * 2].update(measurement.x);
        var filteredY = this._kalmanFilters[i * 2 + 1].update(measurement.y);
        
        smoothed[key] = { x: filteredX, y: filteredY };
      }

      this._stableCorners = newCorners;
      this._displayCorners = smoothed;
    }

    /**
     * @private - Returns true when cornerPoints form a strictly convex polygon
     * in the draw order TL → TR → BR → BL.
     *
     * After Kalman smoothing, two adjacent corners can cross if the detector
     * assigned different physical corners to the same label on successive frames.
     * Drawing that crossing quad produces a bowtie that is geometrically invalid
     * and visually confusing.  This guard detects the crossing and signals that
     * the display state should be reset.
     */
    _isCornersConvex(cp) {
      var pts = [cp.topLeftCorner, cp.topRightCorner, cp.bottomRightCorner, cp.bottomLeftCorner];
      var sign = 0;
      for (var i = 0; i < 4; i++) {
        var a = pts[i];
        var b = pts[(i + 1) % 4];
        var c = pts[(i + 2) % 4];
        var cross = (b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x);
        if (Math.abs(cross) < 1e-6) continue; // collinear — skip
        var s = cross > 0 ? 1 : -1;
        if (sign === 0) { sign = s; }
        else if (s !== sign) { return false; }
      }
      return sign !== 0; // all same sign → convex; sign===0 means degenerate
    }

    /**
     * @private - Calculate target overlay opacity based on quality metrics
     * Returns 0-70 where 0 = fully transparent (good quality), 70 = most opaque (poor quality)
     * Good quality = clear view of document, poor quality = white overlay blocking view
     */
    _calculateTargetOverlayOpacity(report) {
      var checks = report.checks;
      var thresholds = this._thresholds;
      
      // Doc size: 0 when good (at/above threshold), 1 when poor (at 0)
      var docSizeThreshold = thresholds.documentSizeMin || 0.30;
      var docSizeValue = checks.documentSize.value;
      var docSizeRatio = Math.max(0, Math.min(1, 1 - (docSizeValue / docSizeThreshold)));
      if (docSizeValue >= docSizeThreshold) docSizeRatio = 0;
      
      // Sharpness: 0 when good, 1 when poor
      var sharpnessThreshold = thresholds.sharpnessMin || 110;
      var sharpnessValue = checks.sharpness.value;
      var sharpnessRatio = Math.max(0, Math.min(1, 1 - (sharpnessValue / sharpnessThreshold)));
      if (sharpnessValue >= sharpnessThreshold) sharpnessRatio = 0;
      
      // Brightness: 0 when good, 1 when poor
      var brightnessThreshold = thresholds.brightnessMin || 89;
      var brightnessValue = checks.brightness.value;
      var brightnessRatio = Math.max(0, Math.min(1, 1 - (brightnessValue / brightnessThreshold)));
      if (brightnessValue >= brightnessThreshold) brightnessRatio = 0;
      
      // Weighted combination: docSize 50%, sharpness 25%, brightness 25%
      var combinedRatio = docSizeRatio * 0.50 + sharpnessRatio * 0.25 + brightnessRatio * 0.25;
      
      // Map to opacity: 0% (good) to 70% (poor)
      return combinedRatio * 70;
    }

    /**
     * @private - Draw a rounded polygon path (counter-clockwise for hole)
     * @param {CanvasRenderingContext2D} ctx
     * @param {Array} corners - Array of {x, y} points in order
     * @param {number} radius - Corner radius in pixels
     */
    _drawRoundedPolygon(ctx, corners, radius) {
      var n = corners.length;
      if (n < 3) return;
      
      for (var i = 0; i < n; i++) {
        var curr = corners[i];
        var next = corners[(i + 1) % n];
        var prev = corners[(i + n - 1) % n];
        
        // Vectors from current corner to neighbors
        var toPrev = { x: prev.x - curr.x, y: prev.y - curr.y };
        var toNext = { x: next.x - curr.x, y: next.y - curr.y };
        
        // Normalize vectors
        var lenPrev = Math.hypot(toPrev.x, toPrev.y);
        var lenNext = Math.hypot(toNext.x, toNext.y);
        if (lenPrev < 1 || lenNext < 1) continue;
        
        toPrev.x /= lenPrev; toPrev.y /= lenPrev;
        toNext.x /= lenNext; toNext.y /= lenNext;
        
        // Limit radius to half the shortest edge
        var maxR = Math.min(lenPrev, lenNext) / 2;
        var r = Math.min(radius, maxR);
        
        // Points where arc starts and ends
        var arcStart = { x: curr.x + toPrev.x * r, y: curr.y + toPrev.y * r };
        var arcEnd = { x: curr.x + toNext.x * r, y: curr.y + toNext.y * r };
        
        if (i === 0) {
          ctx.moveTo(arcStart.x, arcStart.y);
        } else {
          ctx.lineTo(arcStart.x, arcStart.y);
        }
        ctx.arcTo(curr.x, curr.y, arcEnd.x, arcEnd.y, r);
      }
      ctx.closePath();
    }

    /**
     * @private - Draw spotlight effect: dim outside document, highlight inside
     * Uses canvas clipping to create inverse mask effect
     */
    _drawSpotlightOverlay(ctx, cornerPoints, report, frameW, frameH) {
      var tl = cornerPoints.topLeftCorner;
      var tr = cornerPoints.topRightCorner;
      var bl = cornerPoints.bottomLeftCorner;
      var br = cornerPoints.bottomRightCorner;
      if (!tl || !tr || !bl || !br) return;

      // Calculate target overlay opacity and gradually move towards it
      var targetOpacity = this._calculateTargetOverlayOpacity(report);
      
      if (this._currentTransparency < targetOpacity) {
        this._currentTransparency = Math.min(targetOpacity, this._currentTransparency + this._transparencyStep);
      } else if (this._currentTransparency > targetOpacity) {
        this._currentTransparency = Math.max(targetOpacity, this._currentTransparency - this._transparencyStep);
      }
      
      // Corner radius (scales with document size)
      var docWidth = Math.hypot(tr.x - tl.x, tr.y - tl.y);
      var cornerRadius = Math.max(10, docWidth * 0.035);  // 3.5% of width, min 10px
      
      // Draw dark overlay on everything EXCEPT document area
      // Use evenodd fill rule: outer rect + inner polygon = inverse mask
      ctx.save();
      ctx.beginPath();
      // Outer rectangle (clockwise)
      ctx.moveTo(0, 0);
      ctx.lineTo(frameW, 0);
      ctx.lineTo(frameW, frameH);
      ctx.lineTo(0, frameH);
      ctx.closePath();
      // Inner document polygon with rounded corners (counter-clockwise for hole)
      this._drawRoundedPolygon(ctx, [tl, bl, br, tr], cornerRadius);
      ctx.fillStyle = 'rgba(0, 0, 0, 0.5)';
      ctx.fill('evenodd');
      ctx.restore();
      
      // Draw white fill on document area - opacity based on quality
      // Good quality = 0% opacity (clear view), poor quality = 70% opacity (obscured)
      var alpha = this._currentTransparency / 100;
      ctx.fillStyle = 'rgba(255, 255, 255, ' + alpha + ')';
      ctx.beginPath();
      this._drawRoundedPolygon(ctx, [tl, tr, br, bl], cornerRadius);
      ctx.fill();
    }
  }

  DocumentAutoCapture.State = State;

  // ===========================================================================
  // Capability detection (unchanged from v1)
  // ===========================================================================

  var CaptureCapability = {
    /**
     * Returns true when the page is running inside any kind of WebView or
     * in-app browser where getUserMedia is typically unavailable or silently fails.
     *
     * Covers:
     *  • Social media in-app browsers  (Facebook, Instagram, Twitter/X, WeChat, …)
     *  • Hybrid framework runtimes     (React Native, Capacitor, Cordova)
     *  • Generic native-app wrappers   (apps that set "WebView" in the UA)
     *  • Android Chrome WebView        ("wv" flag in the UA parenthetical comment)
     *  • iOS WKWebView                 (AppleWebKit present; "Safari/NNN" token absent —
     *                                   real Safari and all iOS browsers must include it)
     */
    isWebView: function () {
      var ua = navigator.userAgent || "";
      // Social / messaging in-app browsers
      if (/FBAN|FBAV|Instagram|Line\/|Twitter\/|Snapchat|LinkedIn\/|MicroMessenger|TelegramBot/i.test(ua)) return true;
      // Hybrid framework runtimes
      if (/ReactNative|Capacitor\/|Cordova\//i.test(ua)) return true;
      // Generic WebView marker set by many native-app wrappers
      if (/WebView/i.test(ua)) return true;
      // Android Chrome WebView: Chrome appends "wv" token inside its UA parenthetical
      if (/\bwv\b/.test(ua)) return true;
      // iOS WKWebView: iPhone/iPad UA contains AppleWebKit but omits the "Safari/NNN"
      // version token that real Safari and every iOS browser is required to append
      if (/iPhone|iPad|iPod/.test(ua) && /AppleWebKit/i.test(ua) && !/Safari\/\d/.test(ua)) return true;
      return false;
    },

    hasGetUserMedia: function () {
      return !!(navigator.mediaDevices && navigator.mediaDevices.getUserMedia);
    },

    /**
     * Returns "auto"  – live getUserMedia stream is available and safe to use.
     *         "file"  – use <input type="file"> with native camera (universal fallback).
     *
     * Note: <input type="file"> is supported everywhere; there is no "none" case.
     */
    detect: function () {
      if (CaptureCapability.hasGetUserMedia() && !CaptureCapability.isWebView()) {
        return "auto";
      }
      return "file";
    }
  };

  // ===========================================================================
  // DocumentCaptureFallback - file input with post-capture quality assessment
  // ===========================================================================

  class DocumentCaptureFallback {
    constructor(options) {
      this._scanner = options.scanner;
      this._container = options.containerElement;
      this._thresholds = options.thresholds || {};
      this._onCapture = options.onCapture;
      this._onQualityReport = options.onQualityReport || function () { };
      this._allowSkip = options.allowSkipQuality || false;
      this._fileInput = null;
      this._built = false;
      this._currentPreviewUrl = null; // blob: URL for current photo; revoked on reset
    }

    start() {
      if (!this._built) {
        this._buildUI();
        this._built = true;
      }
      this._reset();
    }

    stop() {
      if (this._built) {
        this._revokePreviewUrl();
        this._container.innerHTML = "";
        this._built = false;
      }
    }

    /** @private */
    _revokePreviewUrl() {
      if (this._currentPreviewUrl) {
        URL.revokeObjectURL(this._currentPreviewUrl);
        this._currentPreviewUrl = null;
      }
    }

    /** @private */
    _buildUI() {
      var self = this;
      this._container.innerHTML = "";

      this._fileInput = document.createElement("input");
      this._fileInput.type = "file";
      this._fileInput.accept = "image/*";
      // Only set capture="environment" on mobile — on desktop this attribute suppresses
      // the file picker and forces camera-only, which is unwanted behaviour.
      if (/Android|iPhone|iPad|iPod/i.test(navigator.userAgent)) {
        this._fileInput.capture = "environment";
      }
      this._fileInput.style.display = "none";
      this._fileInput.addEventListener("change", function () {
        self._handleFile(this.files);
      });

      this._captureBtn = document.createElement("button");
      this._captureBtn.textContent = "📷 Take Photo of Document";
      this._captureBtn.className = "docusnap-fallback-btn";
      this._captureBtn.addEventListener("click", function () {
        self._fileInput.click();
      });

      this._statusEl = document.createElement("div");
      this._statusEl.className = "docusnap-fallback-status";
      this._previewEl = document.createElement("div");
      this._previewEl.className = "docusnap-fallback-preview";
      this._actionsEl = document.createElement("div");
      this._actionsEl.className = "docusnap-fallback-actions";

      this._container.appendChild(this._fileInput);
      this._container.appendChild(this._captureBtn);
      this._container.appendChild(this._statusEl);
      this._container.appendChild(this._previewEl);
      this._container.appendChild(this._actionsEl);
    }

    /** @private */
    _reset() {
      this._revokePreviewUrl();
      if (this._statusEl) this._statusEl.textContent = "";
      if (this._previewEl) this._previewEl.innerHTML = "";
      if (this._actionsEl) this._actionsEl.innerHTML = "";
      if (this._captureBtn) this._captureBtn.style.display = "";
      if (this._fileInput) this._fileInput.value = "";
    }

    /**
     * @private
     * Entry point when the user selects a file.  Shows a loading spinner while
     * the image is processed, then calls _assessImage() with an EXIF-corrected
     * image source.
     *
     * Strategy:
     *  1. Create a blob: URL for preview display — modern browsers automatically
     *     apply EXIF rotation when rendering blob: URLs in <img> elements.
     *  2. Use createImageBitmap(file, {imageOrientation:'from-image'}) to obtain
     *     EXIF-corrected pixel data for detection and extraction.
     *  3. Fall back to FileReader + HTMLImageElement on older browsers.
     */
    _handleFile(files) {
      if (!files || !files.length) return;
      var self = this;
      var file = files[0];

      // Revoke any previous blob URL and show loading state immediately
      this._revokePreviewUrl();
      this._captureBtn.style.display = "none";
      this._statusEl.innerHTML =
        '<span class="docusnap-fallback-loading-text">Analyzing photo\u2026</span>';
      this._previewEl.innerHTML =
        '<div class="docusnap-fallback-loading">' +
        '<div class="docusnap-fallback-spinner"></div>' +
        '</div>';
      this._actionsEl.innerHTML = "";

      // Blob URL for display; the browser handles EXIF rotation for us
      this._currentPreviewUrl = URL.createObjectURL(file);
      var previewUrl = this._currentPreviewUrl;

      // Try EXIF-corrected bitmap first (Chrome 64+, Firefox 90+, Safari 15.4+)
      if (typeof createImageBitmap === "function") {
        createImageBitmap(file, { imageOrientation: "from-image" })
          .then(function (bitmap) {
            self._assessImage(bitmap, previewUrl);
          })
          .catch(function () {
            // Browser supports createImageBitmap but not the options form (older Safari)
            self._handleFileViaReader(file, previewUrl);
          });
      } else {
        self._handleFileViaReader(file, previewUrl);
      }
    }

    /** @private — FileReader + HTMLImageElement fallback (no EXIF correction) */
    _handleFileViaReader(file, previewUrl) {
      var self = this;
      var reader = new FileReader();
      reader.onload = function (e) {
        var img = new Image();
        img.onload = function () { self._assessImage(img, previewUrl); };
        img.src = e.target.result;
      };
      reader.readAsDataURL(file);
    }

    /**
     * @private
     * @param {ImageBitmap|HTMLImageElement} imageSource  EXIF-corrected image
     * @param {string}                       previewUrl   blob: URL for <img> display
     *
     * Downscales large phone photos to ≤1920 px on the longest edge before running
     * the Hough detector (avoids multi-second stalls on 12 MP images).  Corner points
     * from the downscaled detection are scaled back to full resolution for extraction.
     */
    async _assessImage(imageSource, previewUrl) {
      var self = this;
      var MAX_DETECT_DIM = 1920;

      try {
        var srcW = imageSource.naturalWidth  || imageSource.width;
        var srcH = imageSource.naturalHeight || imageSource.height;

        // --- Downscale for detection if the photo is large ---
        var scale = Math.min(1, MAX_DETECT_DIM / Math.max(srcW, srcH));
        var detW  = Math.round(srcW * scale);
        var detH  = Math.round(srcH * scale);

        var detCanvas = document.createElement("canvas");
        detCanvas.width  = detW;
        detCanvas.height = detH;
        var detCtx = detCanvas.getContext("2d");
        detCtx.drawImage(imageSource, 0, 0, detW, detH);
        var imgData = detCtx.getImageData(0, 0, detW, detH);

        // --- Detect document corners on the (possibly downscaled) image ---
        var detection   = this._scanner._detector.detect(imgData.data, detW, detH);
        var detCorners  = detection ? detection.cornerPoints : null;

        // Scale corner coordinates back to full-resolution space for extraction
        var fullCorners = null;
        if (detCorners) {
          if (scale < 1) {
            var inv = 1 / scale;
            fullCorners = {
              topLeftCorner:     { x: detCorners.topLeftCorner.x     * inv, y: detCorners.topLeftCorner.y     * inv },
              topRightCorner:    { x: detCorners.topRightCorner.x    * inv, y: detCorners.topRightCorner.y    * inv },
              bottomRightCorner: { x: detCorners.bottomRightCorner.x * inv, y: detCorners.bottomRightCorner.y * inv },
              bottomLeftCorner:  { x: detCorners.bottomLeftCorner.x  * inv, y: detCorners.bottomLeftCorner.y  * inv },
            };
          } else {
            fullCorners = detCorners;
          }
        }

        // --- Quality assessment on detection-scale data + corners ---
        var report = this._scanner.assessQuality(
          imgData.data, detW, detH, detCorners, this._thresholds
        );
        this._onQualityReport(report);

        // --- Render preview (blob URL; browser auto-applies EXIF rotation) ---
        this._previewEl.innerHTML = "";
        var previewImg = document.createElement("img");
        previewImg.src = previewUrl;
        previewImg.style.maxWidth  = "100%";
        previewImg.style.borderRadius = "8px";
        this._previewEl.appendChild(previewImg);
        this._actionsEl.innerHTML = "";
        this._statusEl.textContent = "";

        if (report.allPassed) {
          this._statusEl.textContent = "✅ Quality check passed";
          this._captureBtn.style.display = "none";

          // --- Perspective-correct extraction using full-resolution corners ---
          var extractedDataUrl = null;
          if (fullCorners) {
            try {
              var cp    = fullCorners;
              var _topW = Math.hypot(cp.topRightCorner.x    - cp.topLeftCorner.x,    cp.topRightCorner.y    - cp.topLeftCorner.y);
              var _botW = Math.hypot(cp.bottomRightCorner.x - cp.bottomLeftCorner.x, cp.bottomRightCorner.y - cp.bottomLeftCorner.y);
              var _lefH = Math.hypot(cp.bottomLeftCorner.x  - cp.topLeftCorner.x,    cp.bottomLeftCorner.y  - cp.topLeftCorner.y);
              var _rigH = Math.hypot(cp.bottomRightCorner.x - cp.topRightCorner.x,   cp.bottomRightCorner.y - cp.topRightCorner.y);
              var _avgW = (_topW + _botW) / 2;
              var _avgH = (_lefH + _rigH) / 2;
              var _longSide = 856;
              var docWidth  = _avgW >= _avgH ? _longSide : Math.round(_longSide * _avgW / _avgH);
              var docHeight = _avgW >= _avgH ? Math.round(_longSide * _avgH / _avgW) : _longSide;
              // imageSource (full-res ImageBitmap or HTMLImageElement) is used here
              var extracted = this._scanner.extractPaper(imageSource, docWidth, docHeight, fullCorners, { margin: 0.25 });
              if (extracted) extractedDataUrl = extracted.toDataURL("image/jpeg", 0.95);
            } catch (e) {
              console.error("docuSnap fallback: extraction error:", e);
            }
          }

          this._onCapture({
            imageData:      previewUrl,
            extractedData:  extractedDataUrl,
            cornerPoints:   fullCorners,
            qualityReport:  report,
          });

        } else {
          var issues = [];
          var checks = report.checks;
          if (!checks.cornersFound.pass)         issues.push("document not fully visible");
          else if (!checks.cornersWithinMargin.pass) issues.push("document too close to edge");
          if (!checks.documentSize.pass)         issues.push("document too small in frame");
          if (!checks.sharpness.pass)            issues.push("image is blurry");
          if (!checks.glare.pass)                issues.push("glare detected");
          if (!checks.brightness.pass)           issues.push("too dark — improve lighting");
          this._statusEl.textContent = "⚠\uFE0F " + issues.join(" · ") + " — please retake.";

          var retryBtn = document.createElement("button");
          retryBtn.textContent = "🔄 Retake Photo";
          retryBtn.className = "docusnap-fallback-btn";
          retryBtn.addEventListener("click", function () {
            self._reset();
            self._fileInput.click();
          });
          this._actionsEl.appendChild(retryBtn);

          if (this._allowSkip) {
            var skipBtn = document.createElement("button");
            skipBtn.textContent = "Use Anyway";
            skipBtn.className = "docusnap-fallback-btn docusnap-fallback-btn-secondary";
            skipBtn.addEventListener("click", function () {
              self._captureBtn.style.display = "none";
              self._actionsEl.innerHTML = "";
              self._statusEl.textContent = "Photo accepted";
              self._onCapture({
                imageData:     previewUrl,
                cornerPoints:  fullCorners,
                qualityReport: report,
              });
            });
            this._actionsEl.appendChild(skipBtn);
          }
        }

      } finally {
        // Free GPU memory held by ImageBitmap (no-op for HTMLImageElement)
        if (imageSource && typeof imageSource.close === "function") {
          imageSource.close();
        }
      }
    }
  }

  // ===========================================================================
  // SmartDocumentCapture - auto-selects the best capture mode
  // ===========================================================================

  class SmartDocumentCapture {
    constructor(options) {
      this._options = options;
      this._mode = null;
      this._instance = null;
      this._onModeSelected = options.onModeSelected || function () { };
    }

    start() {
      var mode = this._options.forceMode || CaptureCapability.detect();
      this._mode = mode;
      this._onModeSelected(mode);

      if (mode === "auto") {
        this._instance = new DocumentAutoCapture(this._options);
        this._instance.start();
      } else {
        this._instance = new DocumentCaptureFallback(this._options);
        this._instance.start();
      }
    }

    stop() {
      if (this._instance) {
        this._instance.stop();
      }
    }

    reset() {
      if (this._instance) {
        if (this._instance.reset) {
          this._instance.reset();
        } else {
          this._instance.stop();
          this._instance.start();
        }
      }
    }

    getMode() {
      return this._mode;
    }
  }

  // ===========================================================================
  // _FaceDetector — internal face presence detector
  //
  // Priority:
  //   1. window.FaceDetector (Shape Detection API — Chrome/Edge, zero bundle cost)
  //   2. pico.js from jsdelivr CDN  (~53 KB pure-JS Viola-Jones, universal fallback)
  //   3. unavailable → returns { present: null }
  //
  // Detection is throttled to every _throttleN frames and scoped to the
  // document bounding box crop to minimise cost and false positives.
  // ===========================================================================

  function _FaceDetector(config) {
    config = config || {};
    this._minConfidence = config.minConfidence != null ? config.minConfidence : 0.6;
    this._throttleN     = 3;       // run every N frames
    this._frameCount    = 0;
    this._lastResult    = null;    // cached between throttled frames
    this._method        = null;    // 'native' | 'pico' | null
    this._nativeDetector = null;
    this._picoCascade   = null;
  }

  _FaceDetector.prototype.isAvailable = function () {
    return this._method !== null;
  };

  /** Try each detection backend in order. Returns true when one is ready. */
  _FaceDetector.prototype.init = async function () {
    // 1. Shape Detection API
    if (typeof window !== 'undefined' && window.FaceDetector) {
      try {
        var fd = new window.FaceDetector({ fastMode: true, maxDetectedFaces: 1 });
        this._nativeDetector = fd;
        this._method = 'native';
        return true;
      } catch (e) { /* Feature-Policy disabled or not supported */ }
    }
    // 2. pico.js from CDN
    try {
      await this._loadPico();
      this._method = 'pico';
      return true;
    } catch (e) {
      console.warn('[DocuSnap] face detection unavailable (pico.js CDN load failed):', e.message);
    }
    this._method = null;
    return false;
  };

  _FaceDetector.prototype._loadPico = function () {
    var self = this;
    return new Promise(function (resolve, reject) {
      var timer = setTimeout(function () { reject(new Error('timeout')); }, 10000);

      function fail(msg) { clearTimeout(timer); reject(new Error(msg)); }

      // Step 1: runtime script
      var script = document.createElement('script');
      script.src = 'https://cdn.jsdelivr.net/gh/nenadmarkus/picojs@c2a16ac/pico.js';
      script.onerror = function () { fail('pico.js script failed'); };
      script.onload = function () {
        if (typeof pico === 'undefined') { return fail('pico not defined'); }
        // Step 2: face cascade (binary file)
        var xhr = new XMLHttpRequest();
        xhr.open('GET', 'https://cdn.jsdelivr.net/gh/nenadmarkus/picojs@c2a16ac/examples/facefinder', true);
        xhr.responseType = 'arraybuffer';
        xhr.onerror = function () { fail('cascade fetch failed'); };
        xhr.onload  = function () {
          try {
            self._picoCascade = pico.unpack_cascade(new Uint8Array(xhr.response));
            clearTimeout(timer);
            resolve();
          } catch (e) { fail(e.message); }
        };
        xhr.send();
      };
      document.head.appendChild(script);
    });
  };

  /**
   * Detect faces in canvas, optionally cropped to the document bounding box.
   * Throttled: returns cached result on non-sampled frames.
   * @param {HTMLCanvasElement} canvas
   * @param {object|null} cropBox  { x, y, width, height } in canvas pixel coords
   * @returns {{ present: bool|null, confidence: number, bounds: object|null }}
   */
  _FaceDetector.prototype.detect = async function (canvas, cropBox) {
    this._frameCount++;
    if (this._frameCount % this._throttleN !== 0) {
      return this._lastResult;  // cached
    }
    if (!this._method) return null;
    try {
      var result = (this._method === 'native')
        ? await this._detectNative(canvas, cropBox)
        : this._detectPico(canvas, cropBox);
      this._lastResult = result;
      return result;
    } catch (e) {
      console.warn('[DocuSnap] face detect error:', e);
      return this._lastResult;
    }
  };

  _FaceDetector.prototype._detectNative = async function (canvas, cropBox) {
    var src = canvas;
    var offsetX = 0, offsetY = 0;
    // Crop to document region to reduce false positives from background
    if (cropBox && cropBox.width > 4 && cropBox.height > 4) {
      var cc = document.createElement('canvas');
      cc.width  = Math.round(cropBox.width);
      cc.height = Math.round(cropBox.height);
      cc.getContext('2d').drawImage(
        canvas,
        cropBox.x, cropBox.y, cropBox.width, cropBox.height,
        0, 0, cc.width, cc.height
      );
      src = cc;
      offsetX = cropBox.x;
      offsetY = cropBox.y;
    }
    var faces = await this._nativeDetector.detect(src);
    if (!faces || faces.length === 0) {
      return { present: false, confidence: 0, bounds: null };
    }
    var bb = faces[0].boundingBox;
    return {
      present:    true,
      confidence: 1.0,
      bounds: { x: bb.x + offsetX, y: bb.y + offsetY, width: bb.width, height: bb.height },
    };
  };

  _FaceDetector.prototype._detectPico = function (canvas, cropBox) {
    if (!this._picoCascade) return null;
    // pico.js expects a greyscale image + the run_cascade function
    var ctx = canvas.getContext('2d');
    var cw = canvas.width, ch = canvas.height;
    var rgbaData = ctx.getImageData(0, 0, cw, ch).data;
    var gray = new Uint8Array(cw * ch);
    for (var i = 0; i < cw * ch; i++) {
      gray[i] = (77 * rgbaData[i * 4] + 150 * rgbaData[i * 4 + 1] + 29 * rgbaData[i * 4 + 2]) >> 8;
    }
    var image = { pixels: gray, nrows: ch, ncols: cw, ldim: cw };
    // Restrict search area to document crop if provided
    var params = {
      shiftfactor: 0.1, minsize: 20, maxsize: 400, scalefactor: 1.1,
    };
    var cascade = this._picoCascade;
    var dets = pico.run_cascade(image, cascade, params, null) || [];
    var minConf = this._minConfidence;
    dets = dets.filter(function (d) { return d[3] > minConf; });
    if (!dets.length) return { present: false, confidence: 0, bounds: null };
    var best = dets.reduce(function (a, b) { return a[3] > b[3] ? a : b; });
    var r = best[0], c = best[1], s = best[2];
    var ox = cropBox ? cropBox.x : 0;
    var oy = cropBox ? cropBox.y : 0;
    return {
      present:    true,
      confidence: Math.min(1, best[3]),
      bounds:     { x: c - s / 2 + ox, y: r - s / 2 + oy, width: s, height: s },
    };
  };

  // ===========================================================================
  // DocuSnap — Clean public API (single integration entry point)
  //
  // Wraps DocumentAutoCapture + DocumentCaptureFallback + _FaceDetector with:
  //   – normalized 0-100 quality values
  //   – multi-side scanning  (sides: 1 | 2)
  //   – face detection       (Shape Detection API + pico.js CDN fallback)
  //   – Blob image output    (not data URLs)
  //   – instructionCode + hintEscalated pattern (inspired by Innovatrics DOT SDK)
  //   – automatic captureMode routing  (smart / auto / file)
  // ===========================================================================

  var InstructionCode = {
    SEARCHING:        'SEARCHING',
    MOVE_CLOSER:      'MOVE_CLOSER',
    HOLD_STILL:       'HOLD_STILL',
    REDUCE_GLARE:     'REDUCE_GLARE',
    IMPROVE_LIGHTING: 'IMPROVE_LIGHTING',
    SHARPEN:          'SHARPEN',
    CENTER_DOCUMENT:  'CENTER_DOCUMENT',
    CONFIRMED:        'CONFIRMED',
  };

  class DocuSnap {
    /**
     * @param {object}   options
     * @param {string}   [options.documentType='any']     'id' | 'passport' | 'document' | 'any'
     * @param {string}   [options.captureMode='smart']    'smart' | 'auto' | 'manual' | 'file'
     * @param {number}   [options.sides=1]                1 | 2
     * @param {Array}    [options.sideConfig]             Per-side overrides [{ documentType, quality, face }]
     * @param {object}   [options.quality]                { sharpness, brightness, glare, size } (all 0-100)
     * @param {object}   [options.face]                   { detect, requirePresent, minConfidence }
     * @param {Element}  [options.fallbackContainer]      Container element for file-capture UI
     * @param {function} options.onCapture                function(CaptureResult)
     * @param {function} [options.onFrame]                function(FrameResult)
     * @param {function} [options.onError]                function(DocuSnapError)
     */
    constructor(options) {
      options = options || {};
      // — Configuration —
      this.documentType      = options.documentType      || 'any';
      this.captureMode       = options.captureMode       || 'smart';
      this.sides             = options.sides             || 1;
      this.sideConfig        = options.sideConfig        || [];
      this.quality           = options.quality           || {};
      this.faceConfig        = options.face              || { detect: false };
      this._fallbackContainer = options.fallbackContainer || null;
      // — Callbacks —
      this._onCapture = options.onCapture || function () {};
      this._onFrame   = options.onFrame   || function () {};
      this._onError   = options.onError   || function () {};
      // — Runtime state —
      this._currentSide       = 0;
      this._capturedSides     = [];
      this._scanState         = 'idle';
      this._actualCaptureMode = null;
      this._videoElement      = null;
      this._canvasElement     = null;
      this._thresholds        = {};
      // — Internal components —
      this._core          = null;   // low-level docuSnap (scanner)
      this._autoCapture   = null;
      this._fallback      = null;
      this._faceDetector  = null;
      // — Hint escalation —
      this._lastCode        = null;
      this._codeRepeatCount = 0;
      this._ESCALATE_AFTER  = 8;  // frames before hint is considered "escalated"
    }

    // ── Lifecycle ─────────────────────────────────────────────────────────────

    /**
     * Initialise and start capture.
     * @param {HTMLVideoElement}  videoElement   Required for auto mode; unused for file mode.
     * @param {HTMLCanvasElement} canvasElement  Display canvas (overlay rendered here).
     * @returns {Promise<void>}
     */
    async start(videoElement, canvasElement) {
      this._scanState     = 'initializing';
      this._currentSide   = 0;
      this._capturedSides = [];
      this._videoElement  = videoElement;
      this._canvasElement = canvasElement;

      // Build thresholds from current side config
      this._thresholds = this._buildThresholds(this._currentSideConfig());

      // Initialise low-level scanner with aspect limits from document type
      var limits = this._aspectLimits(this._currentSideDocType());
      this._core = new docuSnap({
        minAspectRatio: limits.min,
        maxAspectRatio: limits.max,
      });
      await this._core.init();

      // Initialise face detector if any side requires it
      if (this._needsFaceDetection()) {
        this._faceDetector = new _FaceDetector({
          minConfidence: (this.faceConfig.minConfidence != null ? this.faceConfig.minConfidence : 0.6),
        });
        await this._faceDetector.init();
      }

      // Route to correct capture mode.
      // If videoElement is null the caller has no stream — force file mode regardless of
      // what CaptureCapability.detect() says (e.g. after a runtime getUserMedia failure).
      var capability = (videoElement && this.captureMode !== 'file')
        ? CaptureCapability.detect()
        : 'file';
      if (this.captureMode === 'file' || !videoElement) {
        this._actualCaptureMode = 'file';
      } else {
        // 'smart' | 'auto' | 'manual' all fall back to file when camera unavailable
        this._actualCaptureMode = (capability === 'auto') ? 'auto' : 'file';
      }

      this._scanState = 'ready';
      if (this._actualCaptureMode === 'auto') {
        this._startAutoCapture();
      } else {
        this._startFileCapture();
      }
    }

    /** Manually trigger a capture (use when captureMode:'manual'). */
    capture() {
      if (this._autoCapture) {
        this._autoCapture.capture();
      }
    }

    /** Advance to next document side. Call from onCapture when sides > 1. */
    nextSide() {
      if (this._currentSide >= this.sides - 1) return;
      this._currentSide++;
      this._lastCode        = null;
      this._codeRepeatCount = 0;
      this._scanState       = 'ready';
      // Rebuild thresholds for the new side
      this._thresholds = this._buildThresholds(this._currentSideConfig());
      if (this._autoCapture) {
        this._autoCapture._thresholds = this._thresholds;
        this._autoCapture.reset();
      } else if (this._fallback) {
        this._fallback._thresholds = this._thresholds;
        this._fallback.start();
      }
    }

    /** Pause the live-stream capture loop. */
    pause() {
      if (this._autoCapture && this._scanState !== 'captured') {
        this._autoCapture.stop();
        this._scanState = 'paused';
      }
    }

    /** Resume from paused state. */
    resume() {
      if (this._scanState === 'paused' && this._autoCapture) {
        this._scanState = 'ready';
        this._autoCapture.start();
      }
    }

    /** Reset to initial state for the first side. */
    reset() {
      this._currentSide     = 0;
      this._capturedSides   = [];
      this._lastCode        = null;
      this._codeRepeatCount = 0;
      this._scanState       = 'ready';
      if (this._autoCapture) {
        this._autoCapture.reset();
      } else if (this._fallback) {
        this._fallback.start();
      }
    }

    /** Stop capture and release all resources. */
    destroy() {
      this._scanState = 'idle';
      if (this._autoCapture) { this._autoCapture.stop(); this._autoCapture = null; }
      if (this._fallback)    { this._fallback.stop();    this._fallback    = null; }
      this._core         = null;
      this._faceDetector = null;
    }

    // ── Private: setup ─────────────────────────────────────────────────────────

    _startAutoCapture() {
      var self = this;
      var isManual = (this.captureMode === 'manual');
      this._autoCapture = new DocumentAutoCapture({
        scanner:                 this._core,
        videoElement:            this._videoElement,
        canvasElement:           this._canvasElement,
        thresholds:              this._thresholds,
        consecutiveFramesNeeded: isManual ? 5 : 5,
        stayStillDurationMs:     isManual ? 999999 : 1000,  // never auto-fires in manual mode
        frameIntervalMs:         66,
        manualMode:              isManual,
        onFrameData: function (raw) { self._handleFrameData(raw); },
        onCapture:   function (raw) { self._handleAutoCapture(raw); },
        onStateChange:   function () {},   // handled via onFrameData
        onQualityReport: function () {},   // handled via onFrameData
      });
      this._autoCapture.start();
    }

    _startFileCapture() {
      if (!this._fallbackContainer) {
        // Emit a synthetic first frame so the caller knows we're in file mode
        this._onFrame({
          state: 'searching', instructionCode: InstructionCode.SEARCHING,
          hint: 'Tap to take a photo of the document', hintEscalated: false,
          quality: { sharpness: 0, brightness: 0, glare: 0, size: 0, failing: [] },
          corners: null, face: null,
          captureMode: 'file', sideIndex: this._currentSide, sidesTotal: this.sides,
        });
        return;
      }
      var self = this;
      this._fallback = new DocumentCaptureFallback({
        scanner:          this._core,
        containerElement: this._fallbackContainer,
        thresholds:       this._thresholds,
        allowSkipQuality: true,
        onCapture: function (raw) { self._handleFallbackCapture(raw); },
        onQualityReport: function (report) {
          var q = self._normalizeQuality(report);
          self._onFrame({
            state: report.allPassed ? 'confirming' : 'searching',
            instructionCode: report.allPassed ? InstructionCode.CONFIRMED : InstructionCode.SEARCHING,
            hint: report.allPassed ? 'Quality check passed — tap to use' : 'Retake to improve quality',
            hintEscalated: false,
            quality: q, corners: null, face: null,
            captureMode: 'file', sideIndex: self._currentSide, sidesTotal: self.sides,
          });
        },
      });
      this._fallback.start();
    }

    // ── Private: frame handling ────────────────────────────────────────────────

    async _handleFrameData(raw) {
      if (this._scanState === 'idle' || this._scanState === 'captured') return;

      // Instruction code and hint
      var code      = this._deriveInstructionCode(raw.report, raw.state);
      var hint      = this._codeToHint(code);
      var escalated = false;
      if (code === this._lastCode) {
        this._codeRepeatCount++;
        if (this._codeRepeatCount >= this._ESCALATE_AFTER) escalated = true;
      } else {
        this._lastCode        = code;
        this._codeRepeatCount = 0;
      }
      if (raw.state === State.STAY_STILL || raw.state === State.CAPTURED) {
        this._scanState = 'confirming';
      } else {
        this._scanState = 'ready';
      }

      // Normalized quality
      var q = this._normalizeQuality(raw.report);

      // Face detection (throttled, scoped to document crop on the 640px det canvas)
      var faceResult = null;
      if (this._faceDetector) {
        var detected = null;
        if (raw.detCanvas && raw.detCorners) {
          var cropBox = this._cornersToCropBox(raw.detCorners, raw.detCanvas.width, raw.detCanvas.height);
          detected = await this._faceDetector.detect(raw.detCanvas, cropBox);
        }
        // Always emit at least { present: null } when the detector is active so the
        // UI face row stays visible.  detect() can return null on throttled frames
        // (before the first real sample) or when no backend loaded.
        faceResult = detected || { present: null, confidence: null, bounds: null };
      }

      // Gate auto-capture: if face presence is required and face is not yet confirmed,
      // keep resetting the stay-still countdown so capture never fires without a face.
      if (this.face && this.face.requirePresent &&
          raw.state === State.STAY_STILL && this._autoCapture &&
          (!faceResult || !faceResult.present)) {
        this._autoCapture._stayStillStart = performance.now();
      }

      this._onFrame({
        state:           this._scanState,
        instructionCode: code,
        hint:            hint,
        hintEscalated:   escalated,
        quality:         q,
        corners:         raw.fullCorners,
        face:            faceResult,
        captureMode:     this._actualCaptureMode,
        sideIndex:       this._currentSide,
        sidesTotal:      this.sides,
      });
    }

    _handleAutoCapture(rawResult) {
      var self = this;
      this._scanState = 'captured';
      this._buildCaptureResult(rawResult, 'auto').then(function (cr) {
        self._capturedSides.push(cr);
        self._onCapture(cr);
        // Auto-resume the detection loop after 2 s so the preview doesn't freeze
        setTimeout(function () {
          if (self._autoCapture && self._scanState === 'captured') {
            self._autoCapture.reset();
            self._scanState = 'ready';
          }
        }, 2000);
      }).catch(function (err) {
        self._onError({ code: 'CAPTURE_ERROR', message: err.message, cause: err });
      });
    }

    _handleFallbackCapture(rawResult) {
      var self = this;
      this._scanState = 'captured';
      this._buildCaptureResult(rawResult, 'file').then(function (cr) {
        self._capturedSides.push(cr);
        self._onCapture(cr);
      }).catch(function (err) {
        self._onError({ code: 'CAPTURE_ERROR', message: err.message, cause: err });
      });
    }

    // ── Private: result builders ───────────────────────────────────────────────

    async _buildCaptureResult(rawResult, captureMode) {
      var sideIndex  = this._currentSide;
      var isLastSide = sideIndex === this.sides - 1;

      // Convert data URLs / blob URLs to Blobs
      var imageBlob    = rawResult.imageData
        ? await this._toBlob(rawResult.imageData) : null;
      var documentBlob = rawResult.extractedData
        ? await this._toBlob(rawResult.extractedData) : null;

      var q = rawResult.qualityReport
        ? this._normalizeQuality(rawResult.qualityReport)
        : { sharpness: 0, brightness: 0, glare: 0, size: 0, failing: [] };

      var faceResult = this._faceDetector ? this._faceDetector._lastResult : null;

      return {
        image:         imageBlob,
        documentImage: documentBlob,
        corners:       rawResult.cornerPoints || null,
        quality:       q,
        sideIndex:     sideIndex,
        sidesTotal:    this.sides,
        isLastSide:    isLastSide,
        captureMode:   captureMode || this._actualCaptureMode,
        timestamp:     new Date(),
        face:          faceResult,
      };
    }

    // ── Private: helpers ───────────────────────────────────────────────────────

    /** Convert normalized quality config (0-100) to internal raw thresholds. */
    _buildThresholds(sideConf) {
      var q = Object.assign({}, this.quality, (sideConf && sideConf.quality) || {});
      return {
        sharpnessMin:    ((q.sharpness  != null ? q.sharpness  : 40) / 100) * 219,
        brightnessMin:   ((q.brightness != null ? q.brightness : 40) / 100) * 178,
        glareMax:        (q.glare          != null ? q.glare          : 10)  / 100,
        glareThreshold:   q.glareThreshold != null ? q.glareThreshold : 225,
        documentSizeMin: (q.size  != null ? q.size  : 40) / 100,
        cornerMarginPx:  10,
      };
    }

    /** Normalize raw quality report values to 0-100 integers. */
    _normalizeQuality(report) {
      if (!report || !report.checks) {
        return { sharpness: 0, brightness: 0, glare: 0, size: 0, failing: [] };
      }
      var c = report.checks;
      var failing = [];
      if (!c.sharpness.pass)         failing.push('sharpness');
      if (!c.brightness.pass)        failing.push('brightness');
      if (!c.glare.pass)             failing.push('glare');
      if (!c.documentSize.pass)      failing.push('size');
      if (!c.cornersFound.pass)      failing.push('corners');
      return {
        sharpness:  Math.min(100, Math.round((c.sharpness.value  / 219) * 100)),
        brightness: Math.min(100, Math.round((c.brightness.value / 178) * 100)),
        glare:      Math.min(100, Math.round(c.glare.value * 100)),
        size:       Math.min(100, Math.round(c.documentSize.value * 100)),
        failing:    failing,
      };
    }

    _deriveInstructionCode(report, internalState) {
      if (internalState === State.STAY_STILL) return InstructionCode.HOLD_STILL;
      if (!report || !report.checks)          return InstructionCode.SEARCHING;
      var c = report.checks;
      if (!c.cornersFound.pass)        return InstructionCode.SEARCHING;
      if (!c.cornersWithinMargin.pass) return InstructionCode.CENTER_DOCUMENT;
      if (!c.documentSize.pass)        return InstructionCode.MOVE_CLOSER;
      if (!c.sharpness.pass)           return InstructionCode.SHARPEN;
      if (!c.glare.pass)               return InstructionCode.REDUCE_GLARE;
      if (!c.brightness.pass)          return InstructionCode.IMPROVE_LIGHTING;
      return InstructionCode.CONFIRMED;
    }

    _codeToHint(code) {
      var map = {};
      map[InstructionCode.SEARCHING]        = 'Position document in frame';
      map[InstructionCode.MOVE_CLOSER]      = 'Move closer to document';
      map[InstructionCode.HOLD_STILL]       = 'Hold still\u2026';
      map[InstructionCode.REDUCE_GLARE]     = 'Reduce glare \u2014 tilt document slightly';
      map[InstructionCode.IMPROVE_LIGHTING] = 'Too dark \u2014 improve lighting';
      map[InstructionCode.SHARPEN]          = 'Hold steady \u2014 image is blurry';
      map[InstructionCode.CENTER_DOCUMENT]  = 'Move document away from edges';
      map[InstructionCode.CONFIRMED]        = 'Document detected \u2014 hold still';
      return map[code] || 'Position document in frame';
    }

    _aspectLimits(docType) {
      var m = { id: { min: 1.4, max: 1.8 }, passport: { min: 1.2, max: 1.6 },
                document: { min: 1.0, max: 2.0 }, any: { min: 1.2, max: 1.8 } };
      return m[docType] || m.any;
    }

    _currentSideConfig() { return this.sideConfig[this._currentSide] || null; }

    _currentSideDocType() {
      var sc = this._currentSideConfig();
      return (sc && sc.documentType) || this.documentType;
    }

    _needsFaceDetection() {
      if (this.faceConfig && this.faceConfig.detect) return true;
      return this.sideConfig.some(function (sc) { return sc && sc.face && sc.face.detect; });
    }

    /** Compute axis-aligned bounding box of detected corners (in canvas pixel space). */
    _cornersToCropBox(corners, w, h) {
      if (!corners) return null;
      var xs = [corners.topLeftCorner.x, corners.topRightCorner.x,
                corners.bottomRightCorner.x, corners.bottomLeftCorner.x];
      var ys = [corners.topLeftCorner.y, corners.topRightCorner.y,
                corners.bottomRightCorner.y, corners.bottomLeftCorner.y];
      var x  = Math.max(0, Math.min.apply(null, xs));
      var y  = Math.max(0, Math.min.apply(null, ys));
      var x2 = Math.min(w, Math.max.apply(null, xs));
      var y2 = Math.min(h, Math.max.apply(null, ys));
      return { x: x, y: y, width: x2 - x, height: y2 - y };
    }

    /** Convert a data: or blob: URL to a Blob. */
    async _toBlob(url) {
      if (!url) return null;
      if (url.startsWith('blob:')) {
        try { return await (await fetch(url)).blob(); } catch (e) { return null; }
      }
      // data: URL
      var sep  = url.indexOf(',');
      var mime = url.substring(5, url.indexOf(';'));
      var bin  = atob(url.substring(sep + 1));
      var arr  = new Uint8Array(bin.length);
      for (var i = 0; i < bin.length; i++) arr[i] = bin.charCodeAt(i);
      return new Blob([arr], { type: mime });
    }
  }

  /** Static constants for convenient import-free usage. */
  DocuSnap.InstructionCode = InstructionCode;

  /** Returns 'auto' (live camera available) or 'file' (use file-input). */
  DocuSnap.detectCapability = function () { return CaptureCapability.detect(); };

  return {
    DocuSnap: DocuSnap,
  };
});
