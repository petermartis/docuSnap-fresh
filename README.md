# docuSnap

<!-- badges: replace these placeholders with real shields once a registry entry exists -->
![License: Evaluation/Commercial](https://img.shields.io/badge/license-Evaluation%2FCommercial-orange.svg)
![Vanilla JS](https://img.shields.io/badge/zero%20dependencies-vanilla%20JS-yellow.svg)
![UMD](https://img.shields.io/badge/module-UMD%20%2F%20CJS%20%2F%20AMD-lightgrey.svg)

**Client-side JavaScript library for automatic document capture from a live camera feed or a static image. Designed as the capture front-end for OCR and identity-verification pipelines (Microblink, ID R&D, and similar).**

---

## Features

| Capability | Detail |
|---|---|
| Zero dependencies | Pure vanilla JS, single self-contained UMD file (`src/docusnap.js`) |
| Module formats | `window.DocuSnap` (browser script tag), CommonJS, AMD |
| Document detection | Custom Hough-line corner detector with dual-zone Gaussian blur, directional morphology, and connected-component filtering for robust edge detection on textured backgrounds |
| Corner tracking | Kalman-smoothed bounding box (Q=0.01, R=12.0) with inter-frame velocity prediction for 60 fps rendering; spatial-continuity bonus and geometry-validated line tracking across frames |
| Perspective correction | Inverse homography (DLT) with bilinear interpolation; output at native detected dimensions (≤ 4K cap) with configurable margin |
| Quality gate | Real-time checks for sharpness (Laplacian variance), brightness, glare, document size, corner margin — all configurable via 0–100 thresholds |
| Best-frame selection | Rolling buffer of the last 10 full-resolution frames; on capture trigger, the frame with the highest composite score (sharpness 40%, edge confidence 30%, brightness 15%, low glare 15%) is selected |
| Face detection | Shape Detection API (native, where available) with pico.js cascade fallback loaded on demand from CDN |
| Multi-side scanning | 1 or 2 sides; each side independently configurable (document type, quality thresholds, face requirement) |
| Portrait + landscape | `document` and `any` types accept portrait-orientation documents (aspect ratio ≥ 0.55); `id` type enforces landscape only |
| Capture modes | `smart` (auto-detects camera availability), `auto` (fully automatic quality-gated), `manual` (user-triggered shutter), `file` (WebView / file-input fallback) |
| Output | Blob images (full-frame JPEG 0.95 + perspective-corrected crop), normalized quality scores 0–100, face presence + confidence, corner coordinates, capture metadata |
| Mobile-first | Portrait + landscape; requests up to 4K from camera; only downscales if source exceeds 4K |
| Overlay | Live canvas overlay rendered by the library; callers receive clean callbacks with no DOM coupling beyond the `<video>` and `<canvas>` elements |
| Debug visualization | Optional pipeline debug mode renders intermediate stages (Gaussian blur, Sobel gradients, Otsu threshold, Hough lines with tracked lines) on the preview canvas |

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

## The Flow

```
Camera frame (up to 4K)
        │
        ▼
┌───────────────────────────────────────────────────────┐
│ DETECTION  (downscaled to 480 px wide internally)     │
│                                                       │
│  Grayscale                                            │
│   → Dual-zone Gaussian blur                           │
│      • double-pass 5×5 outside center 55% ROI         │
│      • single-pass 5×5 inside center                  │
│   → Sobel edge detection (Otsu auto-threshold × 0.65) │
│   → Directional morphology                            │
│      • horizontal (5×1) + vertical (1×5) line opening │
│      • union of both orientations                     │
│   → Connected-component filter (remove < 80 px blobs) │
│   → Hough line transform (180 angles)                 │
│      • line family classification (A / B clusters)    │
│      • weaker family boost (2.5×)                     │
│   → Quad candidate search (top 10 H × top 10 V)      │
│      • vanishing-point perspective check              │
│      • convexity + diagonal-ratio guard               │
│      • edge-support scoring (H-sides weighted 2×)     │
│      • outer-line preference (wider pair separation)  │
│      • spatial-continuity bonus (tracked line prox.)  │
│      • aspect, area, center scoring                   │
│      • parallelogram corner correction                │
│   → Line tracking with geometry validation            │
│      • parallel pair < 15°, perpendicular > 50°       │
│      • confidence gate ≥ 0.20, decay after 5 misses   │
│   → Kalman smoothing (8 × KalmanFilter1D)             │
│      • Q = 0.01, R = 12.0                             │
│      • inter-frame velocity prediction at 60 fps      │
│   → Quality gate                                      │
│      • Laplacian variance  (sharpness)                │
│      • Mean luminance      (brightness)               │
│      • Bright-pixel ratio  (glare)                    │
│      • Width coverage      (document size)            │
│      • Corner margin check                            │
└───────────────┬───────────────────────────────────────┘
                │ corners + quality report
                ▼
┌───────────────────────────────────────────────────────┐
│ DISPLAY CANVAS  (max 720 px longest side)             │
│  drawImage video directly (no getImageData)           │
│  spotlight overlay (evenodd clip path)                │
│  3-second initial suppression (no bbox on start)      │
│  corners rendered with Kalman + velocity prediction   │
└───────────────┬───────────────────────────────────────┘
                │ state machine
                │  DETECTING → STAY_STILL → CAPTURED
                ▼
┌───────────────────────────────────────────────────────┐
│ AUTOCAPTURE                                           │
│  10 consecutive quality-passing frames required       │
│  Rolling buffer: last 10 full-res frames retained     │
│  Best-frame selection on trigger:                     │
│    sharpness 40% + edge confidence 30%                │
│    + brightness 15% + low glare 15%                   │
│  → extractPaper() on the winning frame                │
│    inverse homography (DLT) + bilinear interpolation  │
│    output = native detected dimensions (≤ 4K cap)     │
│  → Blob (JPEG 0.95) × 2 (full frame + corrected crop)│
└───────────────────────────────────────────────────────┘
                │
                ▼
         onCapture(CaptureResult)
```

---

## Perspective Correction

When a document is captured, its four detected corners are used to compute an inverse homography matrix (Direct Linear Transform). Each pixel in the output image is backward-mapped to the source frame using this matrix and sampled with bilinear interpolation. The result is a geometrically corrected, front-facing view of the document at its native resolution (up to a 4K cap). A 25% margin is added around the document region to preserve surrounding context.

---

## Autocapture Process

1. **Detection** — The Hough-line detector runs at ~10–15 fps on a 480 px-wide downscaled frame. When a valid quad passes all geometric checks (aspect ratio, convexity, perspective consistency), its corners are smoothed through 8 independent Kalman filters (one per x/y coordinate).

2. **Quality gate** — Every frame with detected corners is scored for sharpness, brightness, glare, and document size. If all four checks pass, a consecutive-good-frames counter increments.

3. **Stay-still countdown** — After 10 consecutive passing frames, the library enters a brief hold-still phase to confirm stability.

4. **Best-frame selection** — Throughout detection, a rolling buffer retains the last 10 full-resolution camera frames along with their quality metrics. At capture time, the frame with the highest composite score (sharpness 40%, edge-support confidence 30%, brightness 15%, low glare 15%) is selected — not necessarily the trigger frame.

5. **Extraction** — The winning frame is perspective-corrected using `extractPaper()` and delivered as a high-quality JPEG Blob alongside the uncropped full-frame image.

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

## Document Types & Aspect Ratios

| `documentType` | Min aspect | Max aspect | Portrait allowed | Intended use |
|---|---|---|---|---|
| `'id'` | 1.2 | 1.8 | No | ISO/IEC 7810 ID-1 cards, credit cards |
| `'document'` | 0.55 | 1.6 | Yes | A4 / letter sheets (portrait 0.707 or landscape 1.414) |
| `'any'` | 0.55 | 2.2 | Yes | General-purpose, accepts both orientations |

When portrait is allowed (aspect < 1.0), the rotation-consistency check is automatically skipped to avoid rejecting correctly detected portrait documents.

---

## Quality Thresholds

All values are **0–100**. Internally the library maps them to raw metric ranges.

| Field | Default | What it gates |
|---|---|---|
| `sharpness` | 40 | Laplacian variance (blur detection) |
| `brightness` | 40 | Mean pixel luminance |
| `glare` | 18 | Fraction of overexposed pixels within document quad |
| `size` | 40 | Document width coverage as fraction of frame |

### Detection Scoring Weights

Internally, quad candidates are ranked by a weighted composite score:

| Component | Weight | Description |
|---|---|---|
| Edge support | 0.40 | Fraction of quad edges supported by actual Sobel edges (H-sides weighted 2×) |
| Aspect score | 0.20 | Proximity to known document aspect ratios |
| Outer score | 0.15 | Rewards wider line-pair separation (rejects inner photo borders) |
| Area score | 0.15 | Rewards larger document coverage (`min(areaFrac / 0.45, 1.0)`) |
| Center score | 0.10 | Proximity of quad center to frame center |
| Proximity bonus | +0.20 | Bonus for quads whose lines are within 70 px of previously tracked lines |

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

Evaluation / Commercial — contact peter.martis@gmail.com for licensing terms.
