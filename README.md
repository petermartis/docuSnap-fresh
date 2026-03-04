# docuSnap

<!-- badges: replace these placeholders with real shields once a registry entry exists -->
![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)
![Vanilla JS](https://img.shields.io/badge/zero%20dependencies-vanilla%20JS-yellow.svg)
![UMD](https://img.shields.io/badge/module-UMD%20%2F%20CJS%20%2F%20AMD-lightgrey.svg)

**Client-side JavaScript library for automatic document capture from a live camera feed or a static image. Designed as the capture front-end for OCR and identity-verification pipelines (Microblink, ID R&D, and similar).**

---

## Features

| Capability | Detail |
|---|---|
| Zero dependencies | Pure vanilla JS, single self-contained UMD file (`src/docusnap.js`) |
| Module formats | `window.DocuSnap` (browser script tag), CommonJS, AMD |
| Document detection | Custom Hough-line corner detector; Kalman-smoothed bounding box; perspective correction via inverse homography + bilinear interpolation |
| Quality gate | Real-time checks for sharpness (Laplacian variance), brightness, glare, document size, corner margin — all configurable via 0–100 thresholds |
| Face detection | Shape Detection API (native, where available) with pico.js cascade fallback loaded on demand from CDN |
| Multi-side scanning | 1 or 2 sides; each side independently configurable (document type, quality thresholds, face requirement) |
| Capture modes | `smart` (auto-detects camera availability), `auto` (fully automatic quality-gated), `manual` (user-triggered shutter), `file` (WebView / file-input fallback) |
| Output | Blob images (full-frame JPEG 0.95 + perspective-corrected crop), normalized quality scores 0–100, face presence + confidence, corner coordinates, capture metadata |
| Mobile-first | Portrait + landscape; requests up to 4K from camera; only downscales if source exceeds 4K |
| Overlay | Live canvas overlay rendered by the library; callers receive clean callbacks with no DOM coupling beyond the `<video>` and `<canvas>` elements |

---

## Live Demo

[petermartis.github.io/docuSnap/demo/](https://petermartis.github.io/docuSnap/demo/)

---

## Quick Start

Include `src/docusnap.js` directly — no build step required.

```html
<script src="src/docusnap.js"></script>
<video id="video" playsinline muted></video>
<canvas id="canvas"></canvas>
```

```js
const snap = new DocuSnap({
  captureMode: 'smart',           // auto-routes between camera and file fallback
  sides: 1,
  quality: { sharpness: 40, brightness: 40, glare: 18, size: 40 },
  onCapture: (result) => {
    // result.image         — Blob, full-resolution JPEG frame
    // result.documentImage — Blob, perspective-corrected crop
    // result.corners       — { topLeftCorner, topRightCorner, … }
    // result.quality       — { sharpness, brightness, glare, size, failing[] }
    // result.face          — { present, confidence, bounds } | null
    console.log('Captured', result);
  },
  onFrame: (frame) => {
    // frame.instructionCode — e.g. 'MOVE_CLOSER', 'HOLD_STILL'
    // frame.hint            — human-readable string
    // frame.hintEscalated   — true when same hint has repeated for ~0.5 s
    // frame.quality         — same shape as onCapture quality
    // frame.state           — 'searching' | 'confirming' | 'captured'
  },
});

navigator.mediaDevices.getUserMedia({ video: { facingMode: 'environment' } })
  .then((stream) => {
    const video = document.getElementById('video');
    video.srcObject = stream;
    video.play();
    snap.start(video, document.getElementById('canvas'));
  });
```

For the full API reference see [API.md](API.md).

---

## How It Works

```
Camera frame (up to 4K)
        │
        ▼
┌───────────────────────────────────────────────────────┐
│ DETECTION CANVAS  (max 640 px longest side)           │
│  drawImage → getImageData (~0.6 MB vs 8 MB at 1080p) │
│                                                       │
│  Grayscale → Gaussian blur (3×3)                     │
│           → Sobel edges (Otsu auto-threshold)         │
│           → Hough line transform (180 angles)         │
│           → Line classification (H / V buckets)       │
│           → Quad candidate search                     │
│              • vanishing-point perspective check      │
│              • convexity, diagonal-ratio guard        │
│              • edge-support scoring                   │
│              • parallelogram corner correction        │
│           → Kalman smoothing (8 × KalmanFilter1D)    │
│           → Quality gate                             │
│              • Laplacian variance  (sharpness)        │
│              • Mean luminance      (brightness)       │
│              • Bright-pixel ratio  (glare)            │
│              • Width coverage      (document size)    │
│              • Corner margin check                    │
└───────────────┬───────────────────────────────────────┘
                │ corners (det-space) + quality report
                ▼
┌───────────────────────────────────────────────────────┐
│ DISPLAY CANVAS  (max 720 px longest side)             │
│  drawImage video directly (no getImageData)           │
│  spotlight overlay (evenodd clip path)                │
│  corners scaled from det-space                        │
└───────────────┬───────────────────────────────────────┘
                │ state machine
                │  DETECTING → STAY_STILL → CAPTURED
                ▼
┌───────────────────────────────────────────────────────┐
│ CAPTURE  (full-resolution grab at trigger time)       │
│  full-res frame → extractPaper()                      │
│  inverse homography (DLT) + bilinear interpolation    │
│  output size = native detected dimensions (≤ 4K cap)  │
│  → Blob (JPEG 0.95) × 2  (full frame + crop)         │
└───────────────────────────────────────────────────────┘
                │
                ▼
         onCapture(CaptureResult)
```

---

## Instruction Codes

The `onFrame` callback receives an `instructionCode` from `DocuSnap.InstructionCode`:

| Code | Meaning |
|---|---|
| `SEARCHING` | No document rectangle detected |
| `MOVE_CLOSER` | Document is too small in frame |
| `CENTER_DOCUMENT` | Corners touching or outside frame edges |
| `SHARPEN` | Laplacian variance below threshold (motion blur / out of focus) |
| `REDUCE_GLARE` | Too many bright pixels within document bounds |
| `IMPROVE_LIGHTING` | Mean luminance below threshold |
| `HOLD_STILL` | Quality passed; waiting for stay-still countdown |
| `CONFIRMED` | All checks passed |

---

## Multi-Side Scanning

```js
const snap = new DocuSnap({
  sides: 2,
  sideConfig: [
    { documentType: 'id', quality: { sharpness: 50 }, face: { detect: true, requirePresent: true } },
    { documentType: 'id', quality: { sharpness: 50 } },
  ],
  onCapture: (result) => {
    console.log(`Side ${result.sideIndex + 1} of ${result.sidesTotal}`);
    if (!result.isLastSide) snap.nextSide();
  },
  // ...
});
```

---

## Quality Thresholds

All values are **0–100**. Internally the library maps them to raw metric ranges.

| Field | Default | What it gates |
|---|---|---|
| `sharpness` | 40 | Laplacian variance (blur detection) |
| `brightness` | 40 | Mean pixel luminance |
| `glare` | 18 | Fraction of overexposed pixels within document quad |
| `size` | 40 | Document width coverage as fraction of frame |

---

## Browser Compatibility

| Browser | Camera capture | File-input fallback | Face detection |
|---|---|---|---|
| Chrome / Edge 88+ | Yes | Yes | Shape Detection API (native) |
| Firefox 90+ | Yes | Yes | pico.js CDN fallback |
| Safari 15.4+ | Yes | Yes | pico.js CDN fallback |
| iOS Safari (real browser) | Yes | Yes | pico.js CDN fallback |
| Android WebView / in-app browser | No (auto-detected) | Yes | pico.js CDN fallback |
| iOS WKWebView / in-app browser | No (auto-detected) | Yes | pico.js CDN fallback |

`DocuSnap.detectCapability()` returns `'auto'` or `'file'` at runtime so you can branch before calling `start()`.

---

## License

MIT — see [LICENSE](LICENSE).
