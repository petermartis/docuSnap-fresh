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
    this.R = measurementNoise != null ? measurementNoise : 12.0;  // Measurement noise (heavier smoothing, less jitter)
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
    this._minWidthFraction = options.minWidthFraction != null ? options.minWidthFraction : 0.30;  // 30% minimum — rejects sub-card blobs while still detecting at natural holding distances
    this._maxAreaFraction = options.maxAreaFraction != null ? options.maxAreaFraction : 0.85;
    this._processWidth = options.processWidth || 480;  // Higher res for better edge detection
    // Pre-compute Hough sin/cos tables
    this._houghReady = false;
    this._sinTable = null;
    this._cosTable = null;
    this._numAngles = 180;
    // Debug visualization
    this._debugLayer = 'normal';  // 'normal'|'blur'|'sobel'|'otsu'|'hough'
    this._debugData = null;       // Cached pipeline intermediates
    this._lastSobelMag = null;
    this._lastSobelMaxMag = 0;
    // Line tracking: protect known good lines across frames
    this._knownLines = null;
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

    // 2. Dual-zone Gaussian blur: heavier (two-pass 5x5) outside center ROI,
    //    standard 5x5 in center where card is expected — suppresses desk texture
    var blurred = this._dualZoneBlur(gray, pw, ph);

    // 3. Sobel edge detection with gradient magnitude
    var edges = this._sobelEdges(blurred, pw, ph);

    // 4. Create skin mask and suppress edges in skin regions
    // TODO: Skin mask may be too aggressive - disabled for debugging
    // var skinMask = this._createSkinMask(rgbaSmall, pw, ph);
    // this._suppressEdgesInMask(edges, skinMask, pw, ph);

    // 5. Hough line transform on edge pixels
    var lines = this._houghLines(edges, pw, ph);
    if (this.debug) console.log('[docuSnap] lines found:', lines.length, lines.slice(0,8).map(function(l){ return {theta: Math.round(l.theta*180/Math.PI)+'°', rho: Math.round(l.rho), votes: l.votes}; }));

    // 5b. Protect known good lines from previous frames
    if (this._knownLines && this._knownLines.length > 0) {
      lines = this._protectKnownLines(lines, this._knownLines);
    }

    if (lines.length < 4) { if (this.debug) console.warn('[docuSnap] KILLED: fewer than 4 lines'); return null; }

    // 6. Find rectangle from line intersections (pass edges for support scoring)
    var best = this._findRectangleFromLines(lines, pw, ph, scale, edges);
    if (this.debug && !best) console.warn('[docuSnap] KILLED: no valid quad found (see individual rejections above)');

    // 7. Update known lines cache from successful detection
    //    Gate: only track lines that form geometrically valid parallel pairs.
    //    This prevents a stray hand/desk line from polluting the tracker.
    if (best && best._lines && best.confidence >= 0.20) {
      var _tl = best._lines;
      var DEG = Math.PI / 180;
      // h-pair (indices 0,1) must be roughly parallel
      var hAngDiff = Math.abs(_tl[0].theta - _tl[1].theta);
      // v-pair (indices 2,3) must be roughly parallel
      var vAngDiff = Math.abs(_tl[2].theta - _tl[3].theta);
      if (vAngDiff > Math.PI / 2) vAngDiff = Math.PI - vAngDiff;
      // Also check h vs v are roughly perpendicular (70-110° apart)
      var hvAng = Math.abs(_tl[0].theta - _tl[2].theta);
      if (hvAng > Math.PI / 2) hvAng = Math.PI - hvAng;

      if (hAngDiff < 15 * DEG && vAngDiff < 15 * DEG && hvAng > 50 * DEG) {
        this._knownLines = _tl.slice();
      }
      // else: detection passed but lines aren't geometrically clean — don't track
    } else if (!best) {
      // Decay: clear after N misses to avoid stale tracking
      this._knownLinesMisses = (this._knownLinesMisses || 0) + 1;
      if (this._knownLinesMisses > 5) {
        this._knownLines = null;
        this._knownLinesMisses = 0;
      }
    } else {
      this._knownLinesMisses = 0;
    }

    // Cache pipeline intermediates for debug visualization
    if (this._debugLayer !== 'normal') {
      this._debugData = {
        pw: pw, ph: ph,
        blurred: blurred,
        sobelMag: this._lastSobelMag,
        sobelMaxMag: this._lastSobelMaxMag,
        edges: edges,
        lines: lines,
      };
    }

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

  /** @private - 5x5 Gaussian blur (sigma ~1.0) */
  DocumentDetector.prototype._gaussianBlur5x5 = function (gray, w, h) {
    var out = new Uint8Array(w * h);
    // Kernel (sum=256, shift >>8):
    // [ 1,  4,  6,  4, 1]
    // [ 4, 16, 24, 16, 4]
    // [ 6, 24, 36, 24, 6]
    // [ 4, 16, 24, 16, 4]
    // [ 1,  4,  6,  4, 1]
    for (var y = 2; y < h - 2; y++) {
      for (var x = 2; x < w - 2; x++) {
        var val =
              gray[(y-2)*w+(x-2)]      + 4*gray[(y-2)*w+(x-1)]  + 6*gray[(y-2)*w+x]    + 4*gray[(y-2)*w+(x+1)]  +   gray[(y-2)*w+(x+2)]
          + 4*gray[(y-1)*w+(x-2)]  + 16*gray[(y-1)*w+(x-1)] + 24*gray[(y-1)*w+x]   + 16*gray[(y-1)*w+(x+1)] + 4*gray[(y-1)*w+(x+2)]
          + 6*gray[y*w+(x-2)]     + 24*gray[y*w+(x-1)]      + 36*gray[y*w+x]       + 24*gray[y*w+(x+1)]     + 6*gray[y*w+(x+2)]
          + 4*gray[(y+1)*w+(x-2)] + 16*gray[(y+1)*w+(x-1)]  + 24*gray[(y+1)*w+x]   + 16*gray[(y+1)*w+(x+1)] + 4*gray[(y+1)*w+(x+2)]
          +   gray[(y+2)*w+(x-2)]  + 4*gray[(y+2)*w+(x-1)]  + 6*gray[(y+2)*w+x]    + 4*gray[(y+2)*w+(x+1)]  +   gray[(y+2)*w+(x+2)];
        out[y * w + x] = (val + 128) >> 8;
      }
    }
    // Copy border pixels (2px border)
    for (var y = 0; y < h; y++) {
      for (var x = 0; x < w; x++) {
        if (y < 2 || y >= h - 2 || x < 2 || x >= w - 2) out[y * w + x] = gray[y * w + x];
      }
    }
    return out;
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
   * @private - Dual-zone blur: standard 5×5 on center 55% of frame where the
   * card is expected, double-pass 5×5 (~9×9 effective σ≈1.4) on the outer band
   * to suppress desk/wood texture while keeping card edges sharp.
   */
  DocumentDetector.prototype._dualZoneBlur = function (gray, w, h) {
    // First pass: full-frame 5×5
    var pass1 = this._gaussianBlur5x5(gray, w, h);
    // Second pass on the whole frame (only outer band will be used)
    var pass2 = this._gaussianBlur5x5(pass1, w, h);

    // Define center ROI (55% of frame)
    var cx0 = Math.round(w * 0.225);
    var cy0 = Math.round(h * 0.225);
    var cx1 = Math.round(w * 0.775);
    var cy1 = Math.round(h * 0.775);

    // Composite: center keeps single-pass, outside uses double-pass
    var out = new Uint8Array(w * h);
    for (var y = 0; y < h; y++) {
      for (var x = 0; x < w; x++) {
        var idx = y * w + x;
        if (x >= cx0 && x <= cx1 && y >= cy0 && y <= cy1) {
          out[idx] = pass1[idx];
        } else {
          out[idx] = pass2[idx];
        }
      }
    }
    return out;
  };

  /**
   * @private - Sobel edge detection with clamped Otsu and morphological cleanup.
   * Returns binary edge map (Uint8Array: 255=edge, 0=not).
   *
   * Pipeline:
   *  1. Sobel gradient magnitude (|gx| + |gy|)
   *  2. Clamp extreme magnitudes so text edges don't dominate Otsu
   *  3. Otsu on clamped histogram with proportional floor + 0.8× relaxation
   *  4. Morphological closing (bridge gaps) then opening (remove dots/text)
   */
  DocumentDetector.prototype._sobelEdges = function (gray, w, h) {
    var mag = new Uint16Array(w * h);
    var maxMag = 0;

    // 1. Compute Sobel gradient magnitude
    for (var y = 1; y < h - 1; y++) {
      for (var x = 1; x < w - 1; x++) {
        var gx = -gray[(y-1)*w+(x-1)] - 2*gray[y*w+(x-1)] - gray[(y+1)*w+(x-1)]
                + gray[(y-1)*w+(x+1)] + 2*gray[y*w+(x+1)] + gray[(y+1)*w+(x+1)];
        var gy = -gray[(y-1)*w+(x-1)] - 2*gray[(y-1)*w+x] - gray[(y-1)*w+(x+1)]
                + gray[(y+1)*w+(x-1)] + 2*gray[(y+1)*w+x] + gray[(y+1)*w+(x+1)];
        var m = (gx < 0 ? -gx : gx) + (gy < 0 ? -gy : gy);
        mag[y * w + x] = m;
        if (m > maxMag) maxMag = m;
      }
    }

    // 2. Clamp extreme magnitudes so text/high-contrast edges don't skew Otsu
    var clampVal = 800;

    // 3. ROI-limited Otsu on center band where document border is expected
    //    Avoids strong hand/background edges dominating the threshold
    var margin = Math.round(w * 0.1);
    var roi = { x0: margin, y0: margin, x1: w - margin - 1, y1: h - margin - 1 };
    var threshold = this._otsuThresholdROI(mag, w, h, clampVal, roi);
    threshold *= 0.65;  // balanced — suppress texture but keep weak card edges

    // 4. Binarize
    var edges = new Uint8Array(w * h);
    for (var i = 0; i < w * h; i++) {
      edges[i] = mag[i] >= threshold ? 255 : 0;
    }

    // 5. Directional morphology: opening with line SEs favors long straight
    //    edges (card borders) over small isotropic texture blobs (wood grain).
    //    Union of H-opening and V-opening preserves both orientations.
    var openH = this._morphOpenLineH(edges, w, h, 5);
    var openV = this._morphOpenLineV(edges, w, h, 5);
    for (var i = 0; i < w * h; i++) {
      edges[i] = (openH[i] || openV[i]) ? 255 : 0;
    }

    // 6. Connected-component filter: remove blobs < minArea pixels.
    //    Wood grain produces many small blobs; card border is a large component.
    edges = this._removeSmallComponents(edges, w, h, 80);

    // Cache for debug visualization
    this._lastSobelMag = mag;
    this._lastSobelMaxMag = maxMag;
    return edges;
  };

  /**
   * @private - Otsu threshold on gradient magnitudes, clamped to maxVal.
   * Values above clampVal are binned into the top bucket so extreme
   * text edges don't dominate the between-class variance.
   */
  DocumentDetector.prototype._otsuThresholdClamped = function (mag, w, h, clampVal) {
    var numBins = 256;
    var binScale = (numBins - 1) / clampVal;
    var hist = new Int32Array(numBins);
    var total = 0;

    for (var i = 0; i < w * h; i++) {
      var v = mag[i];
      if (v <= 0) continue; // skip zero-gradient border pixels
      var bin = Math.min(numBins - 1, (Math.min(v, clampVal) * binScale) | 0);
      hist[bin]++;
      total++;
    }

    if (total === 0) return 0;

    var sumAll = 0;
    for (var i = 0; i < numBins; i++) sumAll += i * hist[i];

    var wB = 0, sumB = 0;
    var maxVariance = 0, bestThresh = 0;

    for (var t = 0; t < numBins; t++) {
      wB += hist[t];
      if (wB === 0) continue;
      var wF = total - wB;
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

    // Map bin index back to gradient scale
    return bestThresh / binScale;
  };

  /** @private - 3×3 binary morphological dilation (any neighbor = 255 → 255) */
  DocumentDetector.prototype._dilate3x3 = function (bin, w, h) {
    var out = new Uint8Array(w * h);
    for (var y = 1; y < h - 1; y++) {
      for (var x = 1; x < w - 1; x++) {
        if (bin[(y-1)*w+(x-1)] || bin[(y-1)*w+x] || bin[(y-1)*w+(x+1)] ||
            bin[y*w+(x-1)]     || bin[y*w+x]     || bin[y*w+(x+1)] ||
            bin[(y+1)*w+(x-1)] || bin[(y+1)*w+x] || bin[(y+1)*w+(x+1)]) {
          out[y * w + x] = 255;
        }
      }
    }
    return out;
  };

  /** @private - 3×3 binary morphological erosion (all neighbors must be 255) */
  DocumentDetector.prototype._erode3x3 = function (bin, w, h) {
    var out = new Uint8Array(w * h);
    for (var y = 1; y < h - 1; y++) {
      for (var x = 1; x < w - 1; x++) {
        if (bin[(y-1)*w+(x-1)] && bin[(y-1)*w+x] && bin[(y-1)*w+(x+1)] &&
            bin[y*w+(x-1)]     && bin[y*w+x]     && bin[y*w+(x+1)] &&
            bin[(y+1)*w+(x-1)] && bin[(y+1)*w+x] && bin[(y+1)*w+(x+1)]) {
          out[y * w + x] = 255;
        }
      }
    }
    return out;
  };

  /** @private - Morphological closing: dilate then erode. Bridges small gaps. */
  DocumentDetector.prototype._morphClose3x3 = function (bin, w, h) {
    return this._erode3x3(this._dilate3x3(bin, w, h), w, h);
  };

  /** @private - Morphological opening: erode then dilate. Removes small fragments. */
  DocumentDetector.prototype._morphOpen3x3 = function (bin, w, h) {
    return this._dilate3x3(this._erode3x3(bin, w, h), w, h);
  };

  /** @private - Soft 3×3 erosion: keep pixel if >=5 of 9 neighbors are set (majority) */
  DocumentDetector.prototype._erode3x3Soft = function (bin, w, h) {
    var out = new Uint8Array(w * h);
    for (var y = 1; y < h - 1; y++) {
      for (var x = 1; x < w - 1; x++) {
        var count = 0;
        for (var dy = -1; dy <= 1; dy++) {
          for (var dx = -1; dx <= 1; dx++) {
            if (bin[(y + dy) * w + (x + dx)]) count++;
          }
        }
        if (count >= 5) out[y * w + x] = 255;
      }
    }
    return out;
  };

  /** @private - Morphological opening with horizontal line SE of given length.
   *  Erode then dilate horizontally: only structures >= len pixels wide survive. */
  DocumentDetector.prototype._morphOpenLineH = function (bin, w, h, len) {
    var half = (len - 1) >> 1;
    // Erode: pixel survives only if all 'len' horizontal neighbors are set
    var eroded = new Uint8Array(w * h);
    for (var y = 0; y < h; y++) {
      for (var x = half; x < w - half; x++) {
        var allSet = true;
        for (var dx = -half; dx <= half; dx++) {
          if (!bin[y * w + x + dx]) { allSet = false; break; }
        }
        if (allSet) eroded[y * w + x] = 255;
      }
    }
    // Dilate: spread surviving pixels back along the horizontal line
    var out = new Uint8Array(w * h);
    for (var y = 0; y < h; y++) {
      for (var x = half; x < w - half; x++) {
        if (eroded[y * w + x]) {
          for (var dx = -half; dx <= half; dx++) out[y * w + x + dx] = 255;
        }
      }
    }
    return out;
  };

  /** @private - Morphological opening with vertical line SE of given length.
   *  Erode then dilate vertically: only structures >= len pixels tall survive. */
  DocumentDetector.prototype._morphOpenLineV = function (bin, w, h, len) {
    var half = (len - 1) >> 1;
    var eroded = new Uint8Array(w * h);
    for (var y = half; y < h - half; y++) {
      for (var x = 0; x < w; x++) {
        var allSet = true;
        for (var dy = -half; dy <= half; dy++) {
          if (!bin[(y + dy) * w + x]) { allSet = false; break; }
        }
        if (allSet) eroded[y * w + x] = 255;
      }
    }
    var out = new Uint8Array(w * h);
    for (var y = half; y < h - half; y++) {
      for (var x = 0; x < w; x++) {
        if (eroded[y * w + x]) {
          for (var dy = -half; dy <= half; dy++) out[(y + dy) * w + x] = 255;
        }
      }
    }
    return out;
  };

  /**
   * @private - Remove connected components smaller than minArea pixels.
   * Uses flood-fill (BFS) on 4-connected neighbors. Wood texture produces
   * many tiny blobs; card borders form large connected components that survive.
   */
  DocumentDetector.prototype._removeSmallComponents = function (bin, w, h, minArea) {
    var labels = new Int32Array(w * h); // 0 = unlabeled
    var label = 0;
    var areas = [];   // areas[label] = pixel count
    var queue = [];

    for (var y = 0; y < h; y++) {
      for (var x = 0; x < w; x++) {
        var idx = y * w + x;
        if (bin[idx] && !labels[idx]) {
          label++;
          var area = 0;
          queue.length = 0;
          queue.push(idx);
          labels[idx] = label;
          var head = 0;
          while (head < queue.length) {
            var ci = queue[head++];
            area++;
            var cx = ci % w, cy = (ci - cx) / w;
            // 4-connected neighbors
            if (cx > 0     && bin[ci - 1] && !labels[ci - 1]) { labels[ci - 1] = label; queue.push(ci - 1); }
            if (cx < w - 1 && bin[ci + 1] && !labels[ci + 1]) { labels[ci + 1] = label; queue.push(ci + 1); }
            if (cy > 0     && bin[ci - w] && !labels[ci - w]) { labels[ci - w] = label; queue.push(ci - w); }
            if (cy < h - 1 && bin[ci + w] && !labels[ci + w]) { labels[ci + w] = label; queue.push(ci + w); }
          }
          areas[label] = area;
        }
      }
    }

    // Zero out small components
    var out = new Uint8Array(w * h);
    for (var i = 0; i < w * h; i++) {
      if (labels[i] && areas[labels[i]] >= minArea) out[i] = 255;
    }
    return out;
  };

  /**
   * @private - Otsu threshold on ROI of gradient magnitudes, clamped.
   * Only pixels inside roi contribute to the histogram, reducing
   * influence of strong hand/background edges outside the document area.
   */
  DocumentDetector.prototype._otsuThresholdROI = function (mag, w, h, clampVal, roi) {
    var numBins = 256;
    var binScale = (numBins - 1) / clampVal;
    var hist = new Int32Array(numBins);
    var total = 0;

    for (var y = roi.y0; y <= roi.y1; y++) {
      for (var x = roi.x0; x <= roi.x1; x++) {
        var v = mag[y * w + x];
        if (v <= 0) continue;
        var bin = Math.min(numBins - 1, (Math.min(v, clampVal) * binScale) | 0);
        hist[bin]++;
        total++;
      }
    }

    if (total === 0) return 0;

    var sumAll = 0;
    for (var i = 0; i < numBins; i++) sumAll += i * hist[i];

    var wB = 0, sumB = 0;
    var maxVariance = 0, bestThresh = 0;

    for (var t = 0; t < numBins; t++) {
      wB += hist[t];
      if (wB === 0) continue;
      var wF = total - wB;
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
    // Threshold = 20% of expected card width — low enough for short sides
    var expectedCardWidth = w * this._minWidthFraction;
    var minLineLen = 25;
    var peaks = [];
    for (var ai = 0; ai < numAngles; ai++) {
      for (var ri = 0; ri < numRho; ri++) {
        var votes = acc[ai * numRho + ri];
        if (votes >= minLineLen) {
          peaks.push({ theta: ai * Math.PI / numAngles, rho: ri - diag, votes: votes });
        }
      }
    }

    // Sort by raw votes descending
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

    // Parallel twin weighting + orientation-balanced selection
    lines = this._boostAndBalanceLines(lines);

    return lines;
  };

  /**
   * @private - Ensure previously trusted lines are present in the current set.
   * Re-inserts known lines if no current line is close enough to them.
   * Prevents flickering when a border line temporarily loses Hough votes.
   */
  DocumentDetector.prototype._protectKnownLines = function (lines, knownLines) {
    var rhoTol = 10;
    var thetaTol = 3 * Math.PI / 180; // ~3 degrees
    var result = lines.slice();
    var added = 0;

    for (var k = 0; k < knownLines.length && added < 4; k++) {
      var kl = knownLines[k];
      var found = false;
      for (var i = 0; i < result.length; i++) {
        var dTheta = Math.abs(result[i].theta - kl.theta);
        if (dTheta > Math.PI / 2) dTheta = Math.PI - dTheta;
        var dRho = Math.abs(result[i].rho - kl.rho);
        if (dTheta <= thetaTol && dRho <= rhoTol) {
          found = true;
          break;
        }
      }
      if (!found) {
        // Re-insert with preserved votes and family tag
        result.push({
          rho: kl.rho,
          theta: kl.theta,
          votes: kl.votes || 1,
          effVotes: kl.effVotes || kl.votes || 1,
          _family: kl._family || null,
          _protected: true,
        });
        added++;
      }
    }

    if (added > 0) {
      result.sort(function (a, b) { return (b.effVotes || b.votes) - (a.effVotes || a.votes); });
    }

    return result;
  };

  /**
   * @private - Boost and balance lines using relative angle clusters.
   *
   * Instead of absolute H/V buckets, finds the two dominant perpendicular
   * angle families from the lines themselves. The weaker family gets boosted
   * so short document edges compete with long ones regardless of rotation.
   *
   * - rawVotes used for accept/reject (threshold already applied above)
   * - effVotes (boosted) used only for sorting/ranking
   * - Reserved slots: up to 6 per family, rest filled by score
   */
  DocumentDetector.prototype._boostAndBalanceLines = function (lines) {
    if (lines.length < 2) return lines;

    // --- Step 1: build angle histogram to find dominant clusters ---
    var dTheta = 2 * Math.PI / 180; // 2-degree bin width
    var numBins = Math.ceil(Math.PI / dTheta);
    var binVotes = new Float32Array(numBins); // vote-weighted histogram
    var binCounts = new Uint8Array(numBins);
    var binIndices = new Array(lines.length);

    for (var i = 0; i < lines.length; i++) {
      var idx = Math.round((lines[i].theta + Math.PI / 2) / dTheta);
      if (idx < 0) idx = 0;
      if (idx >= numBins) idx = numBins - 1;
      binIndices[i] = idx;
      binVotes[idx] += lines[i].votes;
      binCounts[idx]++;
    }

    // Smooth histogram (±1 bin) for cluster detection
    var smoothVotes = new Float32Array(numBins);
    var smoothCounts = new Uint8Array(numBins);
    for (var b = 0; b < numBins; b++) {
      smoothVotes[b] = binVotes[b];
      smoothCounts[b] = binCounts[b];
      if (b > 0) { smoothVotes[b] += binVotes[b - 1]; smoothCounts[b] += binCounts[b - 1]; }
      if (b < numBins - 1) { smoothVotes[b] += binVotes[b + 1]; smoothCounts[b] += binCounts[b + 1]; }
    }

    // --- Step 2: find top two perpendicular angle peaks ---
    // Peak 1: highest vote-weighted bin
    var peak1Bin = 0;
    for (var b = 1; b < numBins; b++) {
      if (smoothVotes[b] > smoothVotes[peak1Bin]) peak1Bin = b;
    }

    // Peak 2: highest vote-weighted bin that's roughly perpendicular to peak1 (80-100°)
    var perpBinDist = Math.round(90 / (dTheta * 180 / Math.PI)); // ~45 bins for 2° width
    var perpTolBins = Math.round(10 / (dTheta * 180 / Math.PI)); // ±10° tolerance
    var peak2Bin = -1;
    var peak2Score = 0;
    for (var b = 0; b < numBins; b++) {
      var dist = Math.abs(b - peak1Bin);
      if (dist > numBins / 2) dist = numBins - dist; // wrap
      if (dist >= perpBinDist - perpTolBins && dist <= perpBinDist + perpTolBins) {
        if (smoothVotes[b] > peak2Score) {
          peak2Score = smoothVotes[b];
          peak2Bin = b;
        }
      }
    }

    // --- Step 3: classify lines into family A (peak1) and family B (peak2) ---
    var clusterTolBins = Math.round(15 / (dTheta * 180 / Math.PI)); // ±15° membership
    var familyA = []; // dominant (usually long sides)
    var familyB = []; // secondary (usually short sides)
    var other = [];

    for (var i = 0; i < lines.length; i++) {
      var bin = binIndices[i];
      var distA = Math.abs(bin - peak1Bin);
      if (distA > numBins / 2) distA = numBins - distA;
      var distB = peak2Bin >= 0 ? Math.abs(bin - peak2Bin) : 9999;
      if (distB > numBins / 2) distB = numBins - distB;

      if (distA <= clusterTolBins) {
        lines[i]._family = 'A';
        familyA.push(lines[i]);
      } else if (distB <= clusterTolBins) {
        lines[i]._family = 'B';
        familyB.push(lines[i]);
      } else {
        lines[i]._family = null;
        other.push(lines[i]);
      }
    }

    // --- Step 4: compute effVotes with relative boosting ---
    // Boost the weaker family so short sides compete
    var boostFactor = 1.5; // parallel cluster boost
    var weakBoost = 2.5;   // extra boost for the weaker family
    var totalA = 0, totalB = 0;
    for (var i = 0; i < familyA.length; i++) totalA += familyA[i].votes;
    for (var i = 0; i < familyB.length; i++) totalB += familyB[i].votes;
    var weakFamily = totalA <= totalB ? 'A' : 'B';

    for (var i = 0; i < lines.length; i++) {
      var base = lines[i].votes;
      var parallelMul = smoothCounts[binIndices[i]] >= 2 ? boostFactor : 1.0;
      var orientMul = lines[i]._family === weakFamily ? weakBoost : 1.0;
      lines[i].effVotes = base * parallelMul * orientMul;
    }

    // --- Step 5: balanced selection with reserved slots per family ---
    var maxPerFamily = 6;
    familyA.sort(function (a, b) { return b.effVotes - a.effVotes; });
    familyB.sort(function (a, b) { return b.effVotes - a.effVotes; });

    var result = [];
    var used = {};
    var addLine = function (l) {
      var key = l.rho + '_' + l.theta;
      if (!used[key]) { result.push(l); used[key] = true; }
    };

    for (var i = 0; i < familyA.length && i < maxPerFamily; i++) addLine(familyA[i]);
    for (var i = 0; i < familyB.length && i < maxPerFamily; i++) addLine(familyB[i]);

    // Fill remaining by effVotes from all lines
    lines.sort(function (a, b) { return b.effVotes - a.effVotes; });
    for (var i = 0; i < lines.length && result.length < 40; i++) addLine(lines[i]);

    // Final sort by effVotes
    result.sort(function (a, b) { return b.effVotes - a.effVotes; });

    return result;
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

    // Classify lines into two families using _family tags from _boostAndBalanceLines.
    // Falls back to absolute 45°/135° if no family tags are present.
    var hLines = [];
    var vLines = [];

    for (var i = 0; i < lines.length; i++) {
      if (lines[i]._family === 'A') {
        hLines.push(lines[i]);
      } else if (lines[i]._family === 'B') {
        vLines.push(lines[i]);
      } else {
        // Untagged lines: use absolute angle as fallback
        var angleDeg = lines[i].theta * 180 / Math.PI;
        if (angleDeg > 45 && angleDeg < 135) {
          hLines.push(lines[i]);
        } else {
          vLines.push(lines[i]);
        }
      }
    }

    if (this.debug) console.log('[docuSnap] hLines:', hLines.length, 'vLines:', vLines.length);
    if (hLines.length < 2 || vLines.length < 2) { if (this.debug) console.warn('[docuSnap] KILLED: not enough h/v lines'); return null; }
    var _dbg = this.debug;
    var _rej = { hAngle:0, hVP:0, hDist:0, vAngle:0, vVP:0, vDist:0, corners:0, size:0, edgeRatio:0, aspect:0, docSize:0, convex:0, diag:0, angles:0, rotation:0, symmetry:0, edgeSupport:0 };

    // Limit search to top candidates
    if (hLines.length > 10) hLines = hLines.slice(0, 10);
    if (vLines.length > 10) vLines = vLines.slice(0, 10);

    // Known document aspect ratios (landscape and portrait)
    var knownAspects = [1.586, 1.417, 1.294, 0.707, 0.773]; // ID, passport, letter, A4-portrait, letter-portrait

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

            // Opposite sides similar — a flat card under realistic phone-distance
            // perspective can't foreshorten more than ~30% (ratio 0.70).
            var widthRatio = Math.min(topW, botW) / Math.max(topW, botW);
            var heightRatio = Math.min(leftH, rightH) / Math.max(leftH, rightH);
            if (widthRatio < 0.70 || heightRatio < 0.70) { _rej.edgeRatio++; continue; }

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
            
            // 3. Group corner-angle budget check.
            // For a perspective projection of a flat rectangle the deviations
            // from 90° are correlated (they must sum to 0).  Instead of
            // checking each corner independently against a wide band, we
            // limit the TOTAL absolute deviation: sum(|angle_i - 90°|).
            // Real extreme perspective ≈ 80°; impossible grout quads > 100°.
            var maxTotalDeviation = 80;  // degrees — total budget across all 4 corners
            var maxSingleAngleDeviation = 30;  // no single corner beyond 60-120°
            var anglesOk = true;
            var totalDeviation = 0;
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
              var dev = Math.abs(aDeg - 90);
              if (dev > maxSingleAngleDeviation) { anglesOk = false; break; }
              totalDeviation += dev;
            }
            if (!anglesOk || totalDeviation > maxTotalDeviation) { _rej.angles++; continue; }

            // --- ROTATION CHECK ---
            // When minAspect < 1.0, portrait orientation is allowed — skip rotation gate.
            // Otherwise document must be roughly horizontal (within ±45°).
            if (self._minAspect >= 1.0) {
              var topDx = sorted[1].x - sorted[0].x;
              var topDy = sorted[1].y - sorted[0].y;
              var rotationRad = Math.atan2(topDy, topDx);
              var rotationDeg = Math.abs(rotationRad * 180 / Math.PI);
              if (rotationDeg > 45 && rotationDeg < 135) { _rej.rotation++; continue; }
            }

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
            // areaScore rewards larger documents (peaks at ~45% of frame area = typical held-card fill).
            // Old tightness (1 - areaFrac) was backwards: it rewarded small blobs over full cards.
            var areaScore = Math.min(areaFrac / 0.45, 1.0);

            // Outer-line preference: prefer line pairs with greater separation.
            // Inner features (photo border, text blocks) produce parallel lines
            // closer together; the true card border is the outermost pair.
            // hSep/vSep → 1.0 when lines span most of the frame dimension.
            var hSep = Math.min(hDist / (ph * 0.6), 1.0);
            var vSep = Math.min(vDist / (pw * 0.6), 1.0);
            var outerScore = (hSep + vSep) / 2;

            var score = edgeSupport * 0.40
                      + aspectScore * 0.20
                      + areaScore   * 0.15
                      + centerScore * 0.10
                      + outerScore  * 0.15;

            // Tracking proximity bonus: when we have known lines from a
            // previous frame, boost quads whose lines are close to them.
            // This prevents the tracker from jumping to a distant line
            // just because it got a few more votes this frame.
            if (self._knownLines && self._knownLines.length === 4) {
              var quadLines = [h1, h2, v1, v2];
              var totalProx = 0;
              for (var ki = 0; ki < 4; ki++) {
                var kl = self._knownLines[ki];
                // Find closest quad line to this known line
                var bestDist = Infinity;
                for (var qi = 0; qi < 4; qi++) {
                  var dRho = Math.abs(quadLines[qi].rho - kl.rho);
                  var dTheta = Math.abs(quadLines[qi].theta - kl.theta);
                  if (dTheta > Math.PI / 2) dTheta = Math.PI - dTheta;
                  var dist = dRho + dTheta * 50; // weight theta heavily
                  if (dist < bestDist) bestDist = dist;
                }
                // Proximity score: 1.0 when exact match, 0.0 when > 70px away
                totalProx += Math.max(0, 1.0 - bestDist / 70);
              }
              var proxScore = totalProx / 4; // average over 4 known lines
              score += proxScore * 0.20; // up to 0.20 bonus for continuity
            }

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
                _lines: [h1, h2, v1, v2],
              };
            }
          }
        }
      }
    }

    this._lastRejections = _rej;
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
    var maxGapPenalty = 0;
    var sideSupports = [0, 0, 0, 0];

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

      sideSupports[si] = supported / (numSamples + 1);
      minSupport = Math.min(minSupport, sideSupports[si]);

      // Track worst gap across all sides (as fraction of side length)
      var gapFraction = maxGap / (numSamples + 1);
      if (gapFraction > maxGapPenalty) maxGapPenalty = gapFraction;
    }

    // Weighted average: horizontal sides (top=0, bottom=2) are the reliable
    // anchor — weight them 2× vs vertical sides (right=1, left=3).
    // H-lines on an ID are longer and more consistently detected.
    var wH = 2.0, wV = 1.0;
    var weightedSupport = (sideSupports[0] * wH + sideSupports[1] * wV
                         + sideSupports[2] * wH + sideSupports[3] * wV)
                         / (2 * wH + 2 * wV);
    var baseScore = minSupport < minSupportRequired ? minSupport : weightedSupport;

    // Apply mild gap penalty only for very large gaps
    if (maxGapPenalty > maxGapFraction) {
      var gapPenalty = (maxGapPenalty - maxGapFraction) / (1.0 - maxGapFraction);
      baseScore *= (1.0 - gapPenalty * 0.25);
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
      // For exactly 4 point correspondences, set h9=1 and solve the
      // resulting 8×8 linear system directly.  This is far more robust
      // than the previous inverse-iteration eigenvector approach which
      // required regularisation hacks and often returned near-identity
      // matrices for cards with little perspective distortion.
      //
      // Homography: dx = (h1·sx + h2·sy + h3) / (h7·sx + h8·sy + 1)
      //             dy = (h4·sx + h5·sy + h6) / (h7·sx + h8·sy + 1)
      // Rearranged: h1·sx + h2·sy + h3 - dx·h7·sx - dx·h8·sy = dx
      //             h4·sx + h5·sy + h6 - dy·h7·sx - dy·h8·sy = dy

      var A = [];  // 8×8
      var b = [];  // 8×1
      for (var i = 0; i < 4; i++) {
        var sx = srcPts[i][0], sy = srcPts[i][1];
        var dx = dstPts[i][0], dy = dstPts[i][1];
        A.push([sx, sy, 1, 0, 0, 0, -dx * sx, -dx * sy]);
        b.push(dx);
        A.push([0, 0, 0, sx, sy, 1, -dy * sx, -dy * sy]);
        b.push(dy);
      }

      var h8 = this._gaussSolve(A, b);
      if (!h8) return null;

      // h8 contains [h1..h8]; append h9=1
      h8.push(1);
      return h8;
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

        if (Math.abs(M[col][col]) < 1e-12) {
          return null;  // Singular matrix — no valid homography
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
    assessQuality(rgba, w, h, cornerPoints, thresholds, detConfidence) {
      thresholds = thresholds || {};
      detConfidence = detConfidence || 0;
      var sharpnessMin = thresholds.sharpnessMin !== undefined ? thresholds.sharpnessMin : 100;
      var brightnessMin = thresholds.brightnessMin !== undefined ? thresholds.brightnessMin : 40;
      var glareMax = thresholds.glareMax !== undefined ? thresholds.glareMax : 0.10;
      // Pixel brightness threshold for glare detection (0-255).
      // 248 = only catch near-white hotspots; white card background (~230-247) is not flagged.
      var glareThreshold = thresholds.glareThreshold !== undefined ? thresholds.glareThreshold : 248;
      var documentSizeMin = thresholds.documentSizeMin !== undefined ? thresholds.documentSizeMin : 0.15;
      var cornerMarginPx = thresholds.cornerMarginPx !== undefined ? thresholds.cornerMarginPx : 10;
      // Minimum edge-support confidence from the rectangle detector.
      // 0.15 = detector's internal floor; 0.30 = stronger gate to reject false positives
      // (e.g. bags, clothing edges) that pass sharpness/brightness but aren't real documents.
      var confidenceMin = thresholds.confidenceMin !== undefined ? thresholds.confidenceMin : 0.30;

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
        confidence: { value: detConfidence, pass: detConfidence >= confidenceMin },
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
      this._frameBuffer = [];          // Rolling buffer of last N frames with quality
      this._frameBufferSize = options.frameBufferSize || 10;
      this._lastGoodCorners = null;    // Previous good frame's corners for spatial consistency
      this._cornerDriftMax = 0.12;     // Max 12% drift (fraction of frame diagonal)
      this._stayStillStart = 0;
      this._animFrameId = null;
      this._lastEvalTime = 0;
      this._evaluating = false;

      // Debug visualization state
      this._debugImageDataSmall = null;  // ImageData at processing resolution
      this._debugDataForHough = null;    // Hough data for line drawing
      this._debugCanvas = null;          // Offscreen canvas for upscaling

      // Bounding box smoothing state
      this._stableCorners = null;        // Ground truth corners (validated, display space)
      this._stableFullCorners = null;    // Ground truth corners (full video resolution)
      this._displayCorners = null;       // Currently displayed corners (smoothed)
      this._cornerRejectCount = 0;       // Frames rejecting stable corners
      this._confirmFramesNeeded = 20;    // ~2s of missed frames before clearing the box
      
      // Kalman filter state for each corner (x, y independently)
      // State: [position, velocity], we track 8 values (4 corners x 2 coords)
      this._kalmanFilters = null;        // Array of 8 KalmanFilter1D instances
      
      // Bounding box overlay opacity state (0 = clear, 70 = max overlay)
      this._currentTransparency = 70;    // Start with overlay (will fade as quality improves)
      this._transparencyStep = 5;        // Change by 5% per frame

      // Suppress bounding box drawing for first 3s after session start
      this._sessionStartTime = 0;

      // Cached display state for 60fps rendering (updated by _evaluateFrame)
      this._lastDispW = 0;
      this._lastDispH = 0;
      this._lastReport = null;
      this._lastInstruction = '';        // Current feedback text for canvas overlay
      this._pendingInstruction = '';    // Next text, queued during fade-out
      this._instructionOpacity = 0;     // Current opacity (0–1) for fade animation
      this._instructionFadeTarget = 0;  // Target opacity (0 or 1)
      this._prevInstructionText = '';    // Previous text for fade-out/in transition
    }

    start() {
      // Cancel any rAF that may still be running from the CAPTURED-state drawing
      // loop so we never have two concurrent tick loops.
      if (this._animFrameId) {
        cancelAnimationFrame(this._animFrameId);
        this._animFrameId = null;
      }
      this._state = State.DETECTING;
      this._consecutiveGoodFrames = 0;
      this._candidates = [];
      this._frameBuffer = [];
      this._lastGoodCorners = null;
      this._evaluating = false;
      this._debugImageDataSmall = null;
      this._debugDataForHough = null;
      this._stableCorners = null;
      this._stableFullCorners = null;
      this._displayCorners = null;
      this._cornerRejectCount = 0;
      this._kalmanFilters = null;
      this._currentTransparency = 70;    // Reset overlay opacity
      // Initialize display dimensions from video if available
      if (this._video && this._video.videoWidth) {
        var _vw = this._video.videoWidth, _vh = this._video.videoHeight;
        var _ds = Math.min(1, 720 / Math.max(_vw, _vh));
        this._lastDispW = Math.round(_vw * _ds);
        this._lastDispH = Math.round(_vh * _ds);
      } else {
        this._lastDispW = 0;
        this._lastDispH = 0;
      }
      this._lastReport = null;
      this._lastInstruction = '';
      this._pendingInstruction = '';
      this._instructionOpacity = 0;
      this._instructionFadeTarget = 0;
      this._prevInstructionText = '';
      // Persistent small canvas reused every frame for detection (avoids per-frame allocation)
      if (!this._detCanvas) this._detCanvas = document.createElement("canvas");
      this._sessionStartTime = performance.now(); // suppress bbox for first 3s
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

    /**
     * Set the instruction text shown on the canvas overlay.
     * Call from the onFrame callback with throttled/mapped text.
     * Pass empty string to fade out the instruction.
     */
    setCanvasInstruction(text) {
      text = text || '';
      if (text === this._lastInstruction && text === this._pendingInstruction) return;
      if (!text) {
        // Fade out current text
        this._pendingInstruction = '';
        this._instructionFadeTarget = 0;
      } else if (!this._lastInstruction || this._instructionOpacity < 0.05) {
        // Nothing showing — set text directly and fade in
        this._lastInstruction = text;
        this._pendingInstruction = text;
        this._instructionFadeTarget = 1;
      } else if (text !== this._lastInstruction) {
        // Different text while something is showing — queue and fade out first
        this._pendingInstruction = text;
        this._instructionFadeTarget = 0;
      }
    }

    reset() {
      this.stop();
      this.start();
    }

    /** Set the active debug visualization layer.
     *  @param {string} layer - 'normal'|'blur'|'sobel'|'otsu'|'hough'
     */
    setDebugLayer(layer) {
      this._scanner._detector._debugLayer = layer || 'normal';
      if (layer === 'normal') {
        this._debugImageDataSmall = null;
        this._debugDataForHough = null;
        this._scanner._detector._debugData = null;
      }
    }

    /** @private - Build RGBA ImageData from the current debug layer */
    _updateDebugImageData() {
      var detector = this._scanner._detector;
      if (detector._debugLayer === 'normal' || !detector._debugData) {
        this._debugImageDataSmall = null;
        this._debugDataForHough = null;
        return;
      }
      var d = detector._debugData;
      var pw = d.pw, ph = d.ph;
      var layer = detector._debugLayer;
      var rgba = new Uint8ClampedArray(pw * ph * 4);

      if (layer === 'blur') {
        for (var i = 0; i < pw * ph; i++) {
          var v = d.blurred[i];
          rgba[i * 4] = v; rgba[i * 4 + 1] = v; rgba[i * 4 + 2] = v; rgba[i * 4 + 3] = 255;
        }
      } else if (layer === 'sobel') {
        var maxM = d.sobelMaxMag || 1;
        for (var i = 0; i < pw * ph; i++) {
          var v = Math.min(255, Math.round(d.sobelMag[i] * 255 / maxM));
          rgba[i * 4] = v; rgba[i * 4 + 1] = v; rgba[i * 4 + 2] = v; rgba[i * 4 + 3] = 255;
        }
      } else if (layer === 'otsu') {
        for (var i = 0; i < pw * ph; i++) {
          var v = d.edges[i];
          rgba[i * 4] = v; rgba[i * 4 + 1] = v; rgba[i * 4 + 2] = v; rgba[i * 4 + 3] = 255;
        }
      } else if (layer === 'hough') {
        // Dim edges as background; lines drawn separately via canvas API
        for (var i = 0; i < pw * ph; i++) {
          var v = d.edges[i] ? 40 : 0;
          rgba[i * 4] = v; rgba[i * 4 + 1] = v; rgba[i * 4 + 2] = v; rgba[i * 4 + 3] = 255;
        }
      }

      this._debugImageDataSmall = new ImageData(rgba, pw, ph);
      if (layer === 'hough') {
        d.knownLines = detector._knownLines || null;
        d.rejections = detector._lastRejections || null;
        this._debugDataForHough = d;
      } else {
        this._debugDataForHough = null;
      }
    }

    /**
     * @private - Draw Hough lines with vote-based color gradient.
     * Shows up to 20 lines. H-lines = solid, V-lines = dashed.
     * Color: red (weak) → green (strong) based on effVotes.
     * Labels: "raw→eff" when boosted, "raw" otherwise. H/V tag appended.
     */
    _drawHoughLines(ctx, data) {
      var lines = data.lines;
      var pw = data.pw, ph = data.ph;
      if (!lines || lines.length === 0) return;

      // Split into family A and B, take top 6 of each by effVotes
      var getScore = function(l) { return l.effVotes != null ? l.effVotes : l.votes; };
      var famA = [], famB = [], famO = [];
      for (var i = 0; i < lines.length; i++) {
        if (lines[i]._family === 'A') famA.push(lines[i]);
        else if (lines[i]._family === 'B') famB.push(lines[i]);
        else famO.push(lines[i]);
      }
      famA.sort(function(a, b) { return getScore(b) - getScore(a); });
      famB.sort(function(a, b) { return getScore(b) - getScore(a); });
      famO.sort(function(a, b) { return getScore(b) - getScore(a); });
      var top = famA.slice(0, 6).concat(famB.slice(0, 6)).concat(famO.slice(0, 2));

      var maxScore = 0, minScore = Infinity;
      for (var i = 0; i < top.length; i++) {
        var s = getScore(top[i]);
        if (s > maxScore) maxScore = s;
        if (s < minScore) minScore = s;
      }
      var range = maxScore - minScore || 1;

      ctx.lineWidth = 1.5;

      for (var i = 0; i < top.length; i++) {
        var line = top[i];
        var isA = line._family === 'A';
        var t = (getScore(line) - minScore) / range;
        var hue = Math.round(t * 120);  // 0=red, 120=green
        ctx.strokeStyle = 'hsl(' + hue + ', 100%, 50%)';
        ctx.globalAlpha = 0.5 + 0.5 * t;
        ctx.setLineDash(isA ? [] : [4, 3]); // solid=A(dominant), dashed=B(weak)

        var cosT = Math.cos(line.theta);
        var sinT = Math.sin(line.theta);
        var x0, y0, x1, y1;

        if (Math.abs(sinT) > 0.001) {
          x0 = 0;
          y0 = line.rho / sinT;
          x1 = pw - 1;
          y1 = (line.rho - x1 * cosT) / sinT;
        } else {
          x0 = line.rho / cosT;
          y0 = 0;
          x1 = x0;
          y1 = ph - 1;
        }

        ctx.beginPath();
        ctx.moveTo(x0, y0);
        ctx.lineTo(x1, y1);
        ctx.stroke();
      }

      ctx.setLineDash([]);
      ctx.globalAlpha = 1.0;

      // Vote count labels
      ctx.font = '9px monospace';
      ctx.textBaseline = 'top';
      for (var i = 0; i < top.length; i++) {
        var line = top[i];
        var t = (getScore(line) - minScore) / range;
        var hue = Math.round(t * 120);
        var cosT = Math.cos(line.theta);
        var sinT = Math.sin(line.theta);
        var mx = pw / 2;
        var my = Math.abs(sinT) > 0.001 ? (line.rho - mx * cosT) / sinT : ph / 2;
        if (my < 0 || my > ph) { mx = 0; my = Math.abs(sinT) > 0.001 ? line.rho / sinT : 0; }
        ctx.fillStyle = 'hsl(' + hue + ', 100%, 70%)';
        var label = line.votes + '';
        if (line.effVotes != null && line.effVotes !== line.votes) {
          label += '→' + Math.round(line.effVotes);
        }
        label += line._family ? ' ' + line._family : ' ?';
        ctx.fillText(label, mx + 2, my + 2);
      }

      // Draw tracked (known) lines in blue
      var known = data.knownLines;
      if (known && known.length > 0) {
        ctx.lineWidth = 2.5;
        ctx.strokeStyle = '#00aaff';
        ctx.globalAlpha = 0.9;
        ctx.setLineDash([6, 3]);
        for (var i = 0; i < known.length; i++) {
          var kl = known[i];
          var cosT = Math.cos(kl.theta);
          var sinT = Math.sin(kl.theta);
          var x0, y0, x1, y1;
          if (Math.abs(sinT) > 0.001) {
            x0 = 0; y0 = kl.rho / sinT;
            x1 = pw - 1; y1 = (kl.rho - x1 * cosT) / sinT;
          } else {
            x0 = kl.rho / cosT; y0 = 0;
            x1 = x0; y1 = ph - 1;
          }
          ctx.beginPath();
          ctx.moveTo(x0, y0);
          ctx.lineTo(x1, y1);
          ctx.stroke();
        }
        ctx.setLineDash([]);
        ctx.globalAlpha = 1.0;
        // Label
        ctx.fillStyle = '#00aaff';
        ctx.font = '10px monospace';
        ctx.fillText('TRACKED: ' + known.length + ' lines', 4, ph - 6);
      }

      // Draw rejection stats overlay
      var rej = data.rejections;
      if (rej) {
        ctx.font = '9px monospace';
        ctx.fillStyle = '#ff8800';
        ctx.globalAlpha = 1.0;
        // Only show non-zero rejections
        var parts = [];
        if (rej.hAngle)   parts.push('hAng:' + rej.hAngle);
        if (rej.hVP)      parts.push('hVP:' + rej.hVP);
        if (rej.hDist)    parts.push('hDist:' + rej.hDist);
        if (rej.vAngle)   parts.push('vAng:' + rej.vAngle);
        if (rej.vVP)      parts.push('vVP:' + rej.vVP);
        if (rej.vDist)    parts.push('vDist:' + rej.vDist);
        if (rej.corners)  parts.push('corn:' + rej.corners);
        if (rej.size)     parts.push('size:' + rej.size);
        if (rej.edgeRatio) parts.push('eRat:' + rej.edgeRatio);
        if (rej.aspect)   parts.push('asp:' + rej.aspect);
        if (rej.docSize)  parts.push('dSiz:' + rej.docSize);
        if (rej.convex)   parts.push('cvx:' + rej.convex);
        if (rej.diag)     parts.push('diag:' + rej.diag);
        if (rej.angles)   parts.push('angl:' + rej.angles);
        if (rej.rotation) parts.push('rot:' + rej.rotation);
        if (rej.symmetry) parts.push('sym:' + rej.symmetry);
        if (rej.edgeSupport) parts.push('eSup:' + rej.edgeSupport);
        var line1 = parts.slice(0, 6).join('  ');
        var line2 = parts.slice(6).join('  ');
        ctx.fillText('REJ: ' + line1, 4, 10);
        if (line2) ctx.fillText('     ' + line2, 4, 20);
      }
    }

    /**
     * Manually trigger a capture immediately.
     * Works in both auto-detect and manual modes.
     * In manual mode (manualMode:true) this is the only way to fire a capture.
     */
    capture() {
      if (this._state === State.IDLE || this._state === State.CAPTURED) return;
      // Use buffered frames if available; otherwise snapshot current video frame
      if (this._frameBuffer.length > 0) {
        this._candidates = this._frameBuffer.slice();
      } else {
        var video = this._video;
        var vw = video.videoWidth;
        var vh = video.videoHeight;
        var manCanvas = document.createElement('canvas');
        manCanvas.width  = vw;
        manCanvas.height = vh;
        manCanvas.getContext('2d').drawImage(video, 0, 0, vw, vh);
        this._candidates = [{
          sharpness:   0,
          fullCorners: this._stableFullCorners || null,
          report:      null,
          canvas:      manCanvas,
        }];
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
        if (self._state === State.IDLE) return;

        // ── Render video + overlay at 60fps ──────────────────────
        self._renderFrame();

        // When captured (waiting for nextSide / user interaction), keep the canvas
        // alive so the video preview stays live during the flip prompt.
        // No quality evaluation and no new captures fire in this state.
        if (self._state === State.CAPTURED) {
          self._tick();
          return;
        }

        // Run detection at the configured interval (default ~10fps)
        if (!self._evaluating && timestamp - self._lastEvalTime >= self._frameIntervalMs) {
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

    /** @private - Render video + bounding box overlay at 60fps (called from _tick) */
    _renderFrame() {
      if (!this._canvas || !this._video || !this._video.videoWidth) return;

      var dispW = this._lastDispW || this._canvas.width;
      var dispH = this._lastDispH || this._canvas.height;
      if (!dispW || !dispH) return;

      // Resize only when dimensions actually change
      if (this._canvas.width !== dispW || this._canvas.height !== dispH) {
        this._canvas.width  = dispW;
        this._canvas.height = dispH;
        this._canvas.style.aspectRatio = dispW + " / " + dispH;
      }

      var ctx = this._canvas.getContext("2d");

      // ── Debug layer rendering ──────────────────────────────────────────
      var debugLayer = this._scanner._detector._debugLayer;
      if (debugLayer !== 'normal' && this._debugImageDataSmall) {
        var small = this._debugImageDataSmall;
        if (!this._debugCanvas) this._debugCanvas = document.createElement('canvas');
        var dc = this._debugCanvas;
        if (dc.width !== small.width || dc.height !== small.height) {
          dc.width = small.width;
          dc.height = small.height;
        }
        var dctx = dc.getContext('2d');
        dctx.putImageData(small, 0, 0);
        if (debugLayer === 'hough' && this._debugDataForHough) {
          this._drawHoughLines(dctx, this._debugDataForHough);
        }
        ctx.imageSmoothingEnabled = false;
        ctx.drawImage(dc, 0, 0, dispW, dispH);
        ctx.imageSmoothingEnabled = true;

        // Still show instruction text in debug mode
        var fadeSpeed = 0.042;
        if (this._instructionOpacity < this._instructionFadeTarget) {
          this._instructionOpacity = Math.min(1, this._instructionOpacity + fadeSpeed);
        } else if (this._instructionOpacity > this._instructionFadeTarget) {
          this._instructionOpacity = Math.max(0, this._instructionOpacity - fadeSpeed);
        }
        if (this._instructionOpacity < 0.02 && this._pendingInstruction &&
            this._pendingInstruction !== this._lastInstruction) {
          this._lastInstruction = this._pendingInstruction;
          this._instructionFadeTarget = 1;
        }
        if (this._instructionOpacity > 0.01 && this._lastInstruction) {
          this._drawInstruction(ctx, this._lastInstruction, dispW, dispH, this._instructionOpacity);
        }
        return;  // skip normal video + spotlight rendering
      }

      ctx.drawImage(this._video, 0, 0, dispW, dispH);

      // Don't draw overlay when captured — just show clean video feed
      if (this._state === State.CAPTURED) return;

      // Inter-frame prediction: compute predicted corners for drawing WITHOUT
      // mutating _displayCorners. This prevents velocity drift from accumulating
      // across render frames and avoids bowtie artifacts from corner crossing.
      var drawCorners = this._displayCorners;
      if (this._displayCorners && this._kalmanFilters) {
        var velScale = 16.67 / (this._frameIntervalMs || 66);
        var cornerKeys = ['topLeftCorner', 'topRightCorner', 'bottomLeftCorner', 'bottomRightCorner'];
        drawCorners = {};
        for (var i = 0; i < cornerKeys.length; i++) {
          var key = cornerKeys[i];
          var kfX = this._kalmanFilters[i * 2];
          var kfY = this._kalmanFilters[i * 2 + 1];
          drawCorners[key] = {
            x: this._displayCorners[key].x + kfX.v * velScale,
            y: this._displayCorners[key].y + kfY.v * velScale,
          };
        }

        // Convexity guard: if predicted corners form a bowtie, fall back to
        // raw _displayCorners (which are always from the Kalman filter state)
        if (!this._isCornersConvex(drawCorners)) {
          drawCorners = this._displayCorners;
        }
      }

      // Suppress bounding box for first 3 seconds — let the user stabilize
      var bboxReady = this._sessionStartTime > 0 &&
                      (performance.now() - this._sessionStartTime) >= 3000;

      if (bboxReady && drawCorners && this._lastReport) {
        this._drawSpotlightOverlay(ctx, drawCorners, this._lastReport, dispW, dispH);
      } else {
        ctx.fillStyle = "rgba(0, 0, 0, 0.5)";
        ctx.fillRect(0, 0, dispW, dispH);
      }

      // Animate instruction opacity towards target (~250ms fade)
      var fadeSpeed = 0.042;  // per frame at 60fps ≈ 400ms transition
      if (this._instructionOpacity < this._instructionFadeTarget) {
        this._instructionOpacity = Math.min(1, this._instructionOpacity + fadeSpeed);
      } else if (this._instructionOpacity > this._instructionFadeTarget) {
        this._instructionOpacity = Math.max(0, this._instructionOpacity - fadeSpeed);
      }

      // When fade-out completes and there's pending text, swap and fade in
      if (this._instructionOpacity < 0.02 && this._pendingInstruction &&
          this._pendingInstruction !== this._lastInstruction) {
        this._lastInstruction = this._pendingInstruction;
        this._instructionFadeTarget = 1;
      }

      // Draw instruction text with current opacity
      if (this._instructionOpacity > 0.01 && this._lastInstruction) {
        this._drawInstruction(ctx, this._lastInstruction, dispW, dispH, this._instructionOpacity);
      }
    }

    /** @private - Draw feedback instruction text centered on the canvas */
    _drawInstruction(ctx, text, w, h, opacity) {
      var fontSize = Math.max(14, Math.round(w * 0.035));
      ctx.save();
      ctx.globalAlpha = opacity;
      ctx.font = '600 ' + fontSize + 'px -apple-system, BlinkMacSystemFont, sans-serif';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';

      // Position: lower third of the canvas
      var y = h * 0.85;

      // Semi-transparent pill background
      var metrics = ctx.measureText(text);
      var padX = fontSize * 0.8;
      var padY = fontSize * 0.45;
      var pillW = metrics.width + padX * 2;
      var pillH = fontSize + padY * 2;
      var pillX = (w - pillW) / 2;
      var pillY = y - pillH / 2;
      var radius = pillH / 2;

      ctx.fillStyle = 'rgba(0, 0, 0, 0.65)';
      ctx.beginPath();
      ctx.moveTo(pillX + radius, pillY);
      ctx.lineTo(pillX + pillW - radius, pillY);
      ctx.arcTo(pillX + pillW, pillY, pillX + pillW, pillY + pillH, radius);
      ctx.arcTo(pillX + pillW, pillY + pillH, pillX, pillY + pillH, radius);
      ctx.lineTo(pillX + radius, pillY + pillH);
      ctx.arcTo(pillX, pillY + pillH, pillX, pillY, radius);
      ctx.arcTo(pillX, pillY, pillX + pillW, pillY, radius);
      ctx.closePath();
      ctx.fill();

      // White text
      ctx.fillStyle = '#ffffff';
      ctx.fillText(text, w / 2, y);
      ctx.restore();
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
     * │ CAPTURE    (best buffered full-res frame — corners + pixels matched) │
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
      this._updateDebugImageData();  // build debug viz if active
      var detCorners = detection ? detection.cornerPoints : null;
      var detConfidence = detection ? detection.confidence : 0;

      var report = this._scanner.assessQuality(
        imageData.data, detW, detH, detCorners, this._thresholds, detConfidence
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
      this._updateStableCorners(dispCorners, fullCorners, dispW, dispH);

      // Cache display state for 60fps rendering in _tick
      this._lastDispW  = dispW;
      this._lastDispH  = dispH;
      this._lastReport = report;

      // ── Rolling frame buffer — snapshot passing frames at full resolution ─
      // Only accumulate when quality passes AND corners are spatially consistent.
      if (report.allPassed && fullCorners && this._areCornersConsistent(fullCorners, vw, vh)) {
        var frameCanvas;
        if (this._frameBuffer.length >= this._frameBufferSize) {
          frameCanvas = this._frameBuffer.shift().canvas;
        } else {
          frameCanvas = document.createElement('canvas');
        }
        if (frameCanvas.width !== vw || frameCanvas.height !== vh) {
          frameCanvas.width  = vw;
          frameCanvas.height = vh;
        }
        frameCanvas.getContext('2d').drawImage(video, 0, 0, vw, vh);
        this._frameBuffer.push({
          sharpness:   report.checks.sharpness.value,
          fullCorners: fullCorners,
          report:      report,
          timestamp:   performance.now(),
          canvas:      frameCanvas,
        });
        this._lastGoodCorners = fullCorners;
      }

      // ── State machine ──────────────────────────────────────────────────
      if (this._state === State.DETECTING) {
        if (report.allPassed && fullCorners && this._areCornersConsistent(fullCorners, vw, vh)) {
          this._consecutiveGoodFrames++;
          if (this._consecutiveGoodFrames >= this._consecutiveNeeded) {
            // Enter STAY_STILL in both auto and manual mode.
            // Auto mode: hold for stayStillDurationMs so user sees the bbox.
            // Manual mode: hold until explicit capture() call.
            this._state = State.STAY_STILL;
            this._stayStillStart = performance.now();
            this._onStateChange(State.STAY_STILL, "Hold still...");
          }
        } else {
          this._consecutiveGoodFrames = 0;
          if (!report.allPassed) {
            this._emitInstruction(report);
          }
        }
      }

      // STAY_STILL: quality must stay good; auto mode captures after duration
      if (this._state === State.STAY_STILL) {
        if (!report.allPassed || !fullCorners || !this._areCornersConsistent(fullCorners, vw, vh)) {
          // Quality dropped — reset to DETECTING
          this._state = State.DETECTING;
          this._consecutiveGoodFrames = 0;
          this._frameBuffer = [];
          this._lastGoodCorners = null;
          this._onStateChange(State.DETECTING, "Position document in frame");
        } else if (!this._manualMode) {
          // Auto mode: capture after stayStillDurationMs
          var elapsed = performance.now() - this._stayStillStart;
          if (elapsed >= this._stayStillMs) {
            this._candidates = this._frameBuffer.slice();
            this._selectBestFrame();
          }
        }
        // Manual mode: stay in STAY_STILL until capture() is called
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

      // Pick the best candidate using a composite quality score:
      // sharpness (40%), edge-support confidence (30%), brightness (15%), low glare (15%)
      function _candidateScore(c) {
        if (!c.report || !c.report.checks) return c.sharpness || 0;
        var ch = c.report.checks;
        var sharp = ch.sharpness  ? ch.sharpness.value  / 219 : 0;  // normalize to 0-1
        var conf  = ch.confidence ? ch.confidence.value       : 0;  // already 0-1
        var bright = ch.brightness ? ch.brightness.value / 178 : 0; // normalize to 0-1
        var glare = ch.glare      ? (1 - ch.glare.value)     : 1;  // invert: less glare = better
        return sharp * 0.40 + conf * 0.30 + bright * 0.15 + glare * 0.15;
      }
      var best = this._candidates[0];
      var bestScore = _candidateScore(best);
      for (var i = 1; i < this._candidates.length; i++) {
        var s = _candidateScore(this._candidates[i]);
        if (s > bestScore) { best = this._candidates[i]; bestScore = s; }
      }

      // ── Use the actual buffered frame (corners + pixels are from the same instant) ──
      var capturedImageData = best.canvas.toDataURL("image/jpeg", 0.95);
      var vw = best.canvas.width;
      var vh = best.canvas.height;

      // ── Perspective-correct extraction ────────────────────────────────
      // Decode the JPEG back to an <img> so that extractPaper always receives
      // a same-origin image element — avoids canvas-taint issues that can arise
      // when passing a video-sourced canvas directly to getImageData().
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
          // Use the actual native pixel dimensions of the detected document —
          // never upscale; upscaling adds no real information and hurts OCR.
          // Only downscale if the natural size exceeds the 4K cap recommended
          // by Microblink (3840px on the long side).
          var _MAX_LONG  = 3840;
          var _natLong   = Math.max(_avgW, _avgH);
          var _ds        = _natLong > _MAX_LONG ? _MAX_LONG / _natLong : 1.0;
          var docWidth   = Math.round(_avgW * _ds);
          var docHeight  = Math.round(_avgH * _ds);
          if (_natLong < 1920) {
            console.warn('[DocuSnap] Document long side is ' + Math.round(_natLong) +
              'px — below the 1920px minimum recommended by Microblink / ID R&D.' +
              ' Use a higher-resolution camera for best OCR / liveness results.');
          }
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
      } else if (checks.confidence && !checks.confidence.pass) {
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

    /**
     * @private - Check if current corners are spatially consistent with previous good frame.
     * Returns true if this is the first good frame OR corners haven't drifted more than
     * _cornerDriftMax (fraction of frame diagonal).  Prevents false rectangles (wall edges,
     * clothing) from incrementing the consecutive good frame counter.
     */
    _areCornersConsistent(corners, frameW, frameH) {
      if (!this._lastGoodCorners) return true;  // first good frame — accept
      var prev = this._lastGoodCorners;
      var diag = Math.sqrt(frameW * frameW + frameH * frameH);
      var maxPx = diag * this._cornerDriftMax;
      var keys = ['topLeftCorner', 'topRightCorner', 'bottomRightCorner', 'bottomLeftCorner'];
      for (var i = 0; i < keys.length; i++) {
        var dx = corners[keys[i]].x - prev[keys[i]].x;
        var dy = corners[keys[i]].y - prev[keys[i]].y;
        if (Math.sqrt(dx * dx + dy * dy) > maxPx) {
          // Corner jumped — reset spatial tracking
          this._lastGoodCorners = null;
          this._frameBuffer = [];
          return false;
        }
      }
      return true;
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

    /** @private - Update stable corners with Kalman filter smoothing + 15% clamp */
    _updateStableCorners(newCorners, newFullCorners, frameW, frameH) {
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
          this._stableFullCorners = null;
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
        this._stableFullCorners = newFullCorners;
        this._displayCorners = newCorners;
        return;
      }

      // Update Kalman filters with new measurements
      var cornerKeys = ['topLeftCorner', 'topRightCorner', 'bottomLeftCorner', 'bottomRightCorner'];
      var smoothed = {};
      var maxMove = Math.max(frameW, frameH) * 0.15;  // 15% clamp per detection frame

      for (var i = 0; i < cornerKeys.length; i++) {
        var key = cornerKeys[i];
        var measurement = newCorners[key];

        // Update X and Y filters
        var filteredX = this._kalmanFilters[i * 2].update(measurement.x);
        var filteredY = this._kalmanFilters[i * 2 + 1].update(measurement.y);

        // 15% max change clamp: prevent sudden jumps from noisy detections
        if (this._displayCorners) {
          var prevX = this._displayCorners[key].x;
          var prevY = this._displayCorners[key].y;
          var dx = filteredX - prevX;
          var dy = filteredY - prevY;
          if (Math.abs(dx) > maxMove) filteredX = prevX + Math.sign(dx) * maxMove;
          if (Math.abs(dy) > maxMove) filteredY = prevY + Math.sign(dy) * maxMove;
        }

        smoothed[key] = { x: filteredX, y: filteredY };
      }

      this._stableCorners = newCorners;
      this._stableFullCorners = newFullCorners;
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
              // Native resolution — no upscaling; 4K cap per Microblink guidelines.
              var _MAX_LONG = 3840;
              var _natLong  = Math.max(_avgW, _avgH);
              var _ds       = _natLong > _MAX_LONG ? _MAX_LONG / _natLong : 1.0;
              var docWidth  = Math.round(_avgW * _ds);
              var docHeight = Math.round(_avgH * _ds);
              if (_natLong < 1920) {
                console.warn('[DocuSnap] Document long side is ' + Math.round(_natLong) +
                  'px — below 1920px minimum recommended for OCR / liveness.');
              }
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
    // 1. Shape Detection API (Chrome/Edge — may be behind a flag)
    if (typeof window !== 'undefined' && window.FaceDetector) {
      try {
        var fd = new window.FaceDetector({ fastMode: true, maxDetectedFaces: 1 });
        this._nativeDetector = fd;
        this._method = 'native';
        console.log('[DocuSnap] Face detection: Shape Detection API (native)');
        return true;
      } catch (e) {
        console.log('[DocuSnap] Shape Detection API unavailable:', e.message);
      }
    }
    // 2. pico.js from CDN
    try {
      await this._loadPico();
      this._method = 'pico';
      console.log('[DocuSnap] Face detection: pico.js loaded');
      return true;
    } catch (e) {
      console.warn('[DocuSnap] pico.js failed to load:', e.message);
    }
    this._method = null;
    console.warn('[DocuSnap] No face detection backend available — face results will always be null');
    return false;
  };

  _FaceDetector.prototype._loadPico = function () {
    var self = this;
    return new Promise(function (resolve, reject) {
      var timer = setTimeout(function () { reject(new Error('timeout')); }, 12000);

      function fail(msg) { clearTimeout(timer); reject(new Error(msg)); }

      // Step 1: pico.js runtime from GitHub via jsDelivr (confirmed-working URL)
      var script = document.createElement('script');
      script.src = 'https://cdn.jsdelivr.net/gh/nenadmarkus/picojs/pico.js';
      script.onerror = function () { fail('pico.js script load failed'); };
      script.onload = function () {
        if (typeof pico === 'undefined' || typeof pico.unpack_cascade !== 'function') {
          return fail('pico global not found after script load');
        }
        // Step 2: facefinder cascade — lives in nenadmarkus/pico (C repo), not picojs
        var xhr = new XMLHttpRequest();
        xhr.open('GET', 'https://cdn.jsdelivr.net/gh/nenadmarkus/pico@c2e81f9d23cc11d1a612fd21e4f9de0921a5d0d9/rnt/cascades/facefinder', true);
        xhr.responseType = 'arraybuffer';
        xhr.onerror = function () { fail('cascade fetch failed'); };
        xhr.onload  = function () {
          try {
            // pico.unpack_cascade requires Int8Array (signed bytes)
            self._picoCascade = pico.unpack_cascade(new Int8Array(xhr.response));
            clearTimeout(timer);
            resolve();
          } catch (e) { fail('cascade parse error: ' + e.message); }
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
    // Run on the full canvas — this reliably detects the user's live face in the
    // camera frame. Cropping to the document region was overly restrictive and
    // caused native detection to miss faces that weren't within the corner box.
    var faces = await this._nativeDetector.detect(canvas);
    if (!faces || faces.length === 0) {
      return { present: false, confidence: 0, bounds: null };
    }
    var bb = faces[0].boundingBox;
    return {
      present:    true,
      confidence: 1.0,
      bounds: { x: bb.x, y: bb.y, width: bb.width, height: bb.height },
    };
  };

  _FaceDetector.prototype._detectPico = function (canvas, cropBox) {
    if (!this._picoCascade) return null;

    // When a document cropBox is available, extract that sub-image so pico searches
    // only the document region (finds the face photo on the card) and returns
    // coordinates relative to the full canvas.
    // Without a cropBox (SEARCHING state), scan the whole frame to catch the user's
    // live face in the background.
    var src = canvas;
    var offsetX = 0, offsetY = 0;
    if (cropBox && cropBox.width > 40 && cropBox.height > 40) {
      var cc = document.createElement('canvas');
      cc.width  = Math.round(cropBox.width);
      cc.height = Math.round(cropBox.height);
      cc.getContext('2d').drawImage(
        canvas, cropBox.x, cropBox.y, cropBox.width, cropBox.height,
        0, 0, cc.width, cc.height
      );
      src     = cc;
      offsetX = cropBox.x;
      offsetY = cropBox.y;
    }

    // Convert to greyscale (pico requirement)
    var ctx      = src.getContext('2d');
    var cw       = src.width, ch = src.height;
    var rgbaData = ctx.getImageData(0, 0, cw, ch).data;
    var gray     = new Uint8Array(cw * ch);
    for (var i = 0; i < cw * ch; i++) {
      gray[i] = (77 * rgbaData[i * 4] + 150 * rgbaData[i * 4 + 1] + 29 * rgbaData[i * 4 + 2]) >> 8;
    }

    var image  = { pixels: gray, nrows: ch, ncols: cw, ldim: cw };
    var params = { shiftfactor: 0.1, minsize: 20, maxsize: Math.min(cw, ch) * 0.9, scalefactor: 1.1 };

    var dets = pico.run_cascade(image, this._picoCascade, params) || [];
    // Apply NMS — pico exposes cluster_detections (underscore, not camelCase)
    if (typeof pico.cluster_detections === 'function') {
      dets = pico.cluster_detections(dets, 0.2);
    }
    // Threshold: pico scores for a clear face are typically 5-50+; printed ID card
    // face photos may score lower (2-5), so keep the cutoff permissive.
    dets = dets.filter(function (d) { return d[3] > 2.0; });
    if (!dets.length) return { present: false, confidence: 0, bounds: null };

    var best = dets.reduce(function (a, b) { return a[3] > b[3] ? a : b; });
    var r = best[0], c = best[1], s = best[2];
    return {
      present:    true,
      confidence: Math.min(1, best[3] / 50),
      bounds:     { x: c - s / 2 + offsetX, y: r - s / 2 + offsetY, width: s, height: s },
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

    /** Set the instruction text shown on the canvas overlay (with fade animation). */
    setCanvasInstruction(text) {
      if (this._autoCapture) this._autoCapture.setCanvasInstruction(text);
    }

    /** Set the active debug visualization layer for the detection pipeline.
     *  @param {string} layer - 'normal'|'blur'|'sobel'|'otsu'|'hough'
     */
    setDebugLayer(layer) {
      if (this._autoCapture) this._autoCapture.setDebugLayer(layer);
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

    /** Pause the live-stream capture loop.
     *  Safe to call from any state including from within an onCapture handler. */
    pause() {
      if (this._autoCapture && this._scanState !== 'idle' && this._scanState !== 'paused') {
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
        consecutiveFramesNeeded: isManual ? 10 : 10,
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

      // Face detection (throttled; runs every frame, not only when corners found)
      // When document corners are known, pass the crop box so pico focuses on the
      // document region.  Without corners (SEARCHING state) pass null → scan full frame
      // so the user's live face is already visible before the document is detected.
      var faceResult = null;
      if (this._faceDetector && raw.detCanvas) {
        var cropBox2 = raw.detCorners
          ? this._cornersToCropBox(raw.detCorners, raw.detCanvas.width, raw.detCanvas.height)
          : null;
        var detected = await this._faceDetector.detect(raw.detCanvas, cropBox2);
        // Always emit at least { present: null } when the detector is active so the
        // UI face row stays visible.  detect() can return null on throttled frames
        // (before the first real sample) or when no backend loaded.
        faceResult = detected || { present: null, confidence: null, bounds: null };
      }

      // Gate auto-capture: if face presence is required and face is not yet confirmed,
      // keep resetting the stay-still countdown so capture never fires without a face.
      // The global faceConfig.requirePresent only applies to side 0 (front) — subsequent
      // sides (e.g. the back of an ID card) never have a face and must not be blocked.
      // A per-side override via sideConfig[n].face.requirePresent enforces it on any side.
      var _sideOverride = this._currentSideConfig();
      var _sideFaceCfg  = (_sideOverride && _sideOverride.face != null) ? _sideOverride.face : null;
      var _requireFace  = (_sideFaceCfg && _sideFaceCfg.requirePresent) ||
                          (this.faceConfig && this.faceConfig.requirePresent && this._currentSide === 0);
      if (_requireFace &&
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

        if (cr.isLastSide) {
          // All sides captured. Leave DocumentAutoCapture in its CAPTURED state
          // so the _tick drawing loop keeps the canvas live — identical to the
          // intermediate-side path below.  start() cancels the old rAF if the
          // integrator calls snap.resume() / snap.reset() to start a new session.
          self._scanState = 'paused';
        } else {
          // Intermediate side captured — pause at the DocuSnap level.
          // DocumentAutoCapture stays in its own CAPTURED state: the _tick loop
          // keeps drawing the live video frame (canvas stays active for the user
          // to see while reading the flip prompt) but will NOT fire another capture.
          // Scanning resumes only when the integrator calls snap.nextSide(), which
          // calls _autoCapture.reset() → increments side → starts fresh detection.
          self._scanState = 'paused';
        }
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
        if (cr.isLastSide) {
          self._scanState = 'paused';
        }
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

      var faceResult = this._faceDetector
        ? (this._faceDetector._lastResult || { present: null, confidence: null, bounds: null })
        : null;

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
        glareMax:        (q.glare          != null ? q.glare          : 18)  / 100,
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
      if (c.confidence && !c.confidence.pass) failing.push('confidence');
      return {
        sharpness:  Math.min(100, Math.round((c.sharpness.value  / 219) * 100)),
        brightness: Math.min(100, Math.round((c.brightness.value / 178) * 100)),
        glare:      Math.min(100, Math.round(c.glare.value * 100)),
        size:       Math.min(100, Math.round(c.documentSize.value * 100)),
        confidence: Math.min(100, Math.round((c.confidence ? c.confidence.value : 0) * 100)),
        failing:    failing,
      };
    }

    _deriveInstructionCode(report, internalState) {
      if (internalState === State.STAY_STILL) return InstructionCode.HOLD_STILL;
      if (!report || !report.checks)          return InstructionCode.SEARCHING;
      var c = report.checks;
      if (!c.cornersFound.pass)        return InstructionCode.SEARCHING;
      if (c.confidence && !c.confidence.pass) return InstructionCode.SEARCHING;
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
      var m = {
        id:       { min: 1.2, max: 1.8 },   // ID cards + passports (credit-card shape)
        document: { min: 0.55, max: 1.6 },  // A4/Letter portrait (0.707) + landscape (1.414)
        any:      { min: 0.55, max: 2.2 },  // relaxed — portrait & landscape
      };
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
