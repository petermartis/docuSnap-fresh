# DocuSnap API Reference

DocuSnap is a zero-dependency, browser-native document capture library. It detects ID cards, passports, and A4 documents in a live camera stream (or via file input as a fallback), applies real-time quality gates, and delivers perspective-corrected image `Blob` objects to your callback.

---

## Table of Contents

1. [Installation](#installation)
2. [Constructor](#constructor)
3. [Instance Methods](#instance-methods)
4. [Static Members](#static-members)
5. [FrameResult](#frameresult)
6. [CaptureResult](#captureresult)
7. [Supporting Types](#supporting-types)
8. [Code Examples](#code-examples)

---

## Installation

### Script tag

```html
<script src="docusnap.js"></script>
<!-- DocuSnap is available on window.DocuSnap -->
```

### CommonJS / Node bundler

```js
const { DocuSnap } = require('./src/docusnap.js');
```

### AMD

```js
define(['docusnap'], ({ DocuSnap }) => { /* ... */ });
```

---

## Constructor

```js
new DocuSnap(options)
```

Creates a new DocuSnap instance. All options are optional except `onCapture`.

### Options

| Option | Type | Default | Description |
|---|---|---|---|
| `documentType` | `'any' \| 'id' \| 'passport' \| 'document'` | `'any'` | Controls the aspect-ratio acceptance window used during document detection. See aspect-ratio ranges below. |
| `captureMode` | `'smart' \| 'auto' \| 'manual' \| 'file'` | `'smart'` | How capture is triggered. `'smart'` auto-selects based on browser capability. `'auto'` uses fully automatic quality-gated capture. `'manual'` requires an explicit call to `snap.capture()`. `'file'` forces the file-input / WebView fallback path regardless of camera availability. |
| `sides` | `1 \| 2` | `1` | Number of document sides to scan sequentially (e.g. front and back of an ID card). |
| `sideConfig` | `SideConfig[]` | `[]` | Per-side configuration overrides. Index 0 = front, index 1 = back. Each entry can selectively override `documentType`, `quality`, and/or `face` for that side only. |
| `quality` | `QualityThresholds` | see below | Quality gate thresholds applied to every processed frame. All values are on a 0–100 scale. |
| `face` | `FaceConfig` | `{ detect: false }` | Face detection configuration applied globally. Per-side overrides take precedence when provided via `sideConfig`. |
| `fallbackContainer` | `HTMLElement \| null` | `null` | A container element into which DocuSnap renders its built-in file-capture UI when operating in file mode. If `null` in file mode, a single synthetic `onFrame` event is emitted and the caller is expected to open a `<input type="file">` independently. |
| `onCapture` | `function(CaptureResult)` | **required** | Called each time a side is successfully captured. Receives a [`CaptureResult`](#captureresult) object. |
| `onFrame` | `function(FrameResult)` | `() => {}` | Called on every processed frame at approximately 10–15 fps. Receives a [`FrameResult`](#frameresult) object. Use this to drive a live UI overlay. |
| `onError` | `function(DocuSnapError)` | `() => {}` | Called for non-fatal errors (e.g. capture pipeline failures). Receives a [`DocuSnapError`](#docusnaperror) object. |

### documentType aspect-ratio ranges

| Value | Min ratio | Max ratio | Intended use |
|---|---|---|---|
| `'id'` | 1.4 | 1.8 | ISO/IEC 7810 ID-1 cards (credit-card size) |
| `'passport'` | 1.2 | 1.6 | ICAO 9303 passport booklets |
| `'document'` | 1.0 | 2.0 | A4 / letter sheets |
| `'any'` | 1.2 | 1.8 | General-purpose default |

### QualityThresholds object

All fields are optional integers on a **0–100 scale**. Omitted fields use the listed defaults.

| Field | Default | Description |
|---|---|---|
| `sharpness` | `40` | Minimum sharpness score. Derived from Laplacian variance; a score below this threshold triggers the `SHARPEN` instruction. |
| `brightness` | `40` | Minimum brightness score. A score below this threshold triggers the `IMPROVE_LIGHTING` instruction. |
| `glare` | `18` | **Maximum** glare score. This is an upper bound — a score above this threshold triggers the `REDUCE_GLARE` instruction. Lower values are stricter. |
| `size` | `40` | Minimum document area as a percentage of the total frame area. A score below this threshold triggers the `MOVE_CLOSER` instruction. |

### FaceConfig object

| Field | Type | Default | Description |
|---|---|---|---|
| `detect` | `boolean` | `false` | Enable face detection. When `true`, a `FaceResult` is included in every `FrameResult` and `CaptureResult`. |
| `requirePresent` | `boolean \| null` | `null` | When `true`, the auto-capture countdown is held until a face is confirmed present. Has no effect when `detect` is `false`. |
| `minConfidence` | `number` (0–1) | `0.6` | Minimum detection confidence for a face to be considered present. |

### SideConfig object

All fields are optional. Present fields override the corresponding top-level option for the specified side only.

| Field | Type | Description |
|---|---|---|
| `documentType` | `string` | Overrides `documentType` for this side. |
| `quality` | `QualityThresholds` | Overrides quality thresholds for this side. Merged on top of the top-level `quality` object. |
| `face` | `FaceConfig` | Overrides face detection config for this side. |

---

## Instance Methods

### `start(videoElement, canvasElement)` → `Promise<void>`

Initialise the scanner and begin capture.

```js
await snap.start(videoEl, canvasEl);
```

| Parameter | Type | Description |
|---|---|---|
| `videoElement` | `HTMLVideoElement \| null` | A `<video>` element whose `srcObject` has been set to a live `MediaStream`. Pass `null` when operating in file mode. |
| `canvasElement` | `HTMLCanvasElement \| null` | The display canvas onto which DocuSnap renders the live overlay (corner highlights, etc.). Pass `null` in file mode. |

When `videoElement` is `null`, or when `captureMode` is `'file'`, DocuSnap automatically routes to the file-input fallback path regardless of what `DocuSnap.detectCapability()` returns.

The returned promise resolves once initialisation is complete and the detection loop (or fallback UI) is running.

---

### `capture()`

Manually trigger a capture. This method is only effective when `captureMode` is `'manual'`. In other modes it is a no-op.

```js
document.getElementById('snapBtn').addEventListener('click', () => snap.capture());
```

When called in manual mode, DocuSnap waits for the current frame to pass the quality gate (sharpness, brightness, glare, size) and then fires `onCapture`. The same `stayStillDuration` countdown applies as in auto mode, so the user has a brief moment to hold steady.

---

### `nextSide()`

Advance to the next document side. Call this from within `onCapture` when `captureResult.isLastSide` is `false`.

```js
onCapture(cr) {
  if (!cr.isLastSide) {
    showFlipPrompt();
    snap.nextSide();
  }
}
```

This increments the internal side counter, rebuilds quality thresholds from the new side's `sideConfig` entry (if any), and resets the detection loop. The `onFrame` callback will continue firing with the updated `sideIndex`.

Calling `nextSide()` when already on the last side is a safe no-op.

---

### `pause()`

Pause the live-stream detection loop. The video stream itself continues; only the frame-processing and capture-triggering are suspended. `onFrame` will stop firing until `resume()` is called.

Has no effect when in file mode or when the current state is already `'captured'`.

---

### `resume()`

Resume processing after a `pause()` call. Restarts the detection loop from the paused state.

---

### `reset()`

Reset to the initial state: side index returns to 0, the candidate queue is cleared, and the detection loop is restarted from the beginning. Useful when the user wants to retake a scan within the same `DocuSnap` instance.

---

### `destroy()`

Stop all processing and release all held resources (timers, internal references). The instance should not be used after calling `destroy()`. Calling `destroy()` does **not** stop the camera `MediaStream` — that is the caller's responsibility.

```js
snap.destroy();
stream.getTracks().forEach(t => t.stop());
```

---

## Static Members

### `DocuSnap.detectCapability()` → `'auto' | 'file'`

Probe the current browser environment and return the appropriate capture mode.

| Return value | Meaning |
|---|---|
| `'auto'` | Live camera capture is available (`getUserMedia` + `requestVideoFrameCallback` or `requestAnimationFrame` are supported). |
| `'file'` | Camera capture is unavailable (e.g. WKWebView on iOS, old Safari, or an environment without `getUserMedia`). Use file-input fallback. |

```js
if (DocuSnap.detectCapability() === 'file') {
  // Show your own file-upload UI before calling snap.start(null, null)
}
```

---

### `DocuSnap.InstructionCode`

A frozen enum object whose values are the string keys used in `FrameResult.instructionCode`. Import this to write `switch`/comparison logic without hardcoding strings.

```js
const IC = DocuSnap.InstructionCode;

// IC.SEARCHING        === 'SEARCHING'
// IC.CONFIRMED        === 'CONFIRMED'
// IC.HOLD_STILL       === 'HOLD_STILL'
// IC.MOVE_CLOSER      === 'MOVE_CLOSER'
// IC.SHARPEN          === 'SHARPEN'
// IC.REDUCE_GLARE     === 'REDUCE_GLARE'
// IC.IMPROVE_LIGHTING === 'IMPROVE_LIGHTING'
// IC.CENTER_DOCUMENT  === 'CENTER_DOCUMENT'
```

| Code | Meaning | Default hint text |
|---|---|---|
| `SEARCHING` | No document detected in the current frame. | `'Position document in frame'` |
| `CONFIRMED` | Document detected and all quality gates passed — capture is imminent. | `'Document detected — hold still'` |
| `HOLD_STILL` | Quality gates passed; stay-still countdown is running before auto-capture fires. | `'Hold still…'` |
| `MOVE_CLOSER` | Document detected but too small relative to the frame. | `'Move closer to document'` |
| `SHARPEN` | Document too blurry (sharpness below threshold). | `'Hold steady — image is blurry'` |
| `REDUCE_GLARE` | Glare fraction exceeds threshold. | `'Reduce glare — tilt document slightly'` |
| `IMPROVE_LIGHTING` | Brightness below threshold. | `'Too dark — improve lighting'` |
| `CENTER_DOCUMENT` | Document corners are too close to the frame edge. | `'Move document away from edges'` |

---

## FrameResult

Received by the `onFrame` callback on every processed frame (~10–15 fps).

| Field | Type | Description |
|---|---|---|
| `instructionCode` | `string` (`InstructionCode`) | The current machine-readable instruction state. Compare against `DocuSnap.InstructionCode`. |
| `hint` | `string` | Human-readable instruction string corresponding to `instructionCode`. Safe to display directly in your UI. |
| `hintEscalated` | `boolean` | `true` when the same instruction code has been emitted for 8 or more consecutive frames. Use this to add visual urgency to the hint (e.g. change colour, add animation). |
| `quality` | [`QualityReport`](#qualityreport) | Live quality scores for the current frame. Scores are only meaningful when `instructionCode !== 'SEARCHING'`. |
| `face` | [`FaceResult`](#faceresult)` \| null` | Face detection result for this frame. `null` when face detection is disabled. |
| `corners` | [`CornerPoints`](#cornerpoints)` \| null` | Detected document corner coordinates in display-canvas pixel space. `null` when no document is found. |
| `captureMode` | `string` | The active capture mode resolved for this session (`'auto'`, `'manual'`, or `'file'`). |
| `sideIndex` | `number` | Zero-based index of the side currently being scanned (`0` = front, `1` = back). |
| `sidesTotal` | `number` | Total number of sides configured. |
| `state` | `string` | Internal scan state string (`'idle'`, `'initializing'`, `'ready'`, `'confirming'`, `'paused'`, `'captured'`). |

---

## CaptureResult

Received by the `onCapture` callback each time a side is successfully captured.

| Field | Type | Description |
|---|---|---|
| `image` | `Blob \| null` | Full-frame JPEG at the native camera resolution (quality 0.95). `null` if image conversion failed. |
| `documentImage` | `Blob \| null` | Perspective-corrected JPEG crop of the document region with a 25% margin added on each side. `null` when no corners were found at capture time. |
| `quality` | [`QualityReport`](#qualityreport) | Quality scores recorded at the moment of capture. |
| `face` | [`FaceResult`](#faceresult)` \| null` | Most recent face detection result at capture time. `null` when face detection is disabled. |
| `corners` | [`CornerPoints`](#cornerpoints)` \| null` | Document corner coordinates at capture time. `null` when corners were not found. |
| `sideIndex` | `number` | Zero-based index of the captured side (`0` = front, `1` = back). |
| `sidesTotal` | `number` | Total number of sides configured. |
| `isLastSide` | `boolean` | `true` when this capture completes the final configured side. When `false`, call `snap.nextSide()` after presenting the intermediate result. |
| `captureMode` | `string` | How this capture was triggered: `'auto'`, `'manual'`, or `'file'`. |
| `timestamp` | `Date` | Wall-clock time when the capture occurred. |

---

## Supporting Types

### QualityReport

Returned in both `FrameResult.quality` and `CaptureResult.quality`.

| Field | Type | Description |
|---|---|---|
| `sharpness` | `number` (0–100) | Laplacian-variance sharpness score normalised to 0–100. |
| `brightness` | `number` (0–100) | Mean pixel brightness normalised to 0–100. |
| `glare` | `number` (0–100) | Fraction of over-exposed pixels within the document region, scaled to 0–100. **Lower is better.** A value above the configured `glare` threshold causes a failure. |
| `size` | `number` (0–100) | Document area as a percentage of the total frame area. |
| `failing` | `string[]` | Array of check names that are currently failing. Possible values: `'sharpness'`, `'brightness'`, `'glare'`, `'size'`, `'corners'`. An empty array means all checks passed. |

---

### FaceResult

| Field | Type | Description |
|---|---|---|
| `present` | `boolean \| null` | Whether a face was detected. `null` when the detector has not yet produced its first result (pending initialisation or throttled frame). |
| `confidence` | `number \| null` | Detection confidence in the range 0–1. `null` when `present` is `null`. |
| `bounds` | `{ x, y, width, height } \| null` | Bounding box of the detected face in detection-canvas pixel space. `null` when no face is present. |

---

### CornerPoints

All coordinates are in display-canvas pixel space (i.e. the coordinate system of the `canvasElement` passed to `start()`).

| Field | Type | Description |
|---|---|---|
| `topLeftCorner` | `{ x: number, y: number }` | Top-left corner of the detected document. |
| `topRightCorner` | `{ x: number, y: number }` | Top-right corner. |
| `bottomRightCorner` | `{ x: number, y: number }` | Bottom-right corner. |
| `bottomLeftCorner` | `{ x: number, y: number }` | Bottom-left corner. |

---

### DocuSnapError

Passed to the `onError` callback.

| Field | Type | Description |
|---|---|---|
| `code` | `string` | Machine-readable error code, e.g. `'CAPTURE_ERROR'`. |
| `message` | `string` | Human-readable description. |
| `cause` | `Error \| null` | The underlying `Error` object, when available. |

---

## Code Examples

### 1. Minimal single-side auto-capture

The simplest possible integration: camera stream, one side, fully automatic.

```html
<video id="video" playsinline style="display:none"></video>
<canvas id="canvas"></canvas>
```

```js
const stream = await navigator.mediaDevices.getUserMedia({ video: { facingMode: 'environment' } });
document.getElementById('video').srcObject = stream;
await document.getElementById('video').play();

const snap = new DocuSnap({
  onCapture(cr) {
    const url = URL.createObjectURL(cr.documentImage || cr.image);
    document.getElementById('result').src = url;
    snap.destroy();
    stream.getTracks().forEach(t => t.stop());
  },
});

await snap.start(
  document.getElementById('video'),
  document.getElementById('canvas')
);
```

---

### 2. Two-side ID card capture with face detection

Scans the front, prompts the user to flip, then scans the back. Face detection is active on the front side only (via `sideConfig`), and capture is blocked until a face is present.

```js
const snap = new DocuSnap({
  documentType: 'id',
  sides: 2,
  sideConfig: [
    // Front: require a face before capture fires
    { face: { detect: true, requirePresent: true, minConfidence: 0.65 } },
    // Back: no face requirement
    { face: { detect: false } },
  ],
  quality: { sharpness: 50, size: 45 },

  onFrame(frame) {
    document.getElementById('hint').textContent = frame.hint;

    if (frame.face) {
      const faceEl = document.getElementById('faceStatus');
      faceEl.textContent = frame.face.present ? 'Face detected' : 'No face';
    }
  },

  onCapture(cr) {
    console.log(`Side ${cr.sideIndex + 1}/${cr.sidesTotal} captured`);

    if (!cr.isLastSide) {
      // Show the front image and prompt the user to flip the card
      showFrontResult(cr.documentImage);
      showFlipPrompt(() => snap.nextSide());
    } else {
      // Both sides complete — send to server
      submitBothSides(capturedSides);
      snap.destroy();
      stream.getTracks().forEach(t => t.stop());
    }

    capturedSides.push(cr);
  },

  onError(err) {
    console.error(err.code, err.message, err.cause);
  },
});

const stream = await navigator.mediaDevices.getUserMedia({ video: { facingMode: 'environment' } });
videoEl.srcObject = stream;
await videoEl.play();
await snap.start(videoEl, canvasEl);
```

---

### 3. Manual capture mode with live quality display

The user sees live quality scores and presses a button to trigger capture themselves.

```js
const snap = new DocuSnap({
  documentType: 'any',
  captureMode: 'manual',

  onFrame(frame) {
    const IC = DocuSnap.InstructionCode;
    const q  = frame.quality;

    // Drive quality meters
    document.getElementById('sharpness').value  = q.sharpness;
    document.getElementById('brightness').value = q.brightness;
    document.getElementById('glare').value      = q.glare;
    document.getElementById('size').value       = q.size;

    // Enable the snap button only when all checks pass
    const ready = frame.instructionCode === IC.CONFIRMED
               || frame.instructionCode === IC.HOLD_STILL;
    document.getElementById('snapBtn').disabled = !ready;

    // Escalate hint colour after 8 consecutive frames with the same warning
    const hintEl = document.getElementById('hint');
    hintEl.textContent  = frame.hint;
    hintEl.style.color  = frame.hintEscalated ? 'orange' : 'white';
  },

  onCapture(cr) {
    const url = URL.createObjectURL(cr.documentImage || cr.image);
    document.getElementById('preview').src = url;
  },
});

document.getElementById('snapBtn').addEventListener('click', () => snap.capture());

const stream = await navigator.mediaDevices.getUserMedia({ video: true });
videoEl.srcObject = stream;
await videoEl.play();
await snap.start(videoEl, canvasEl);
```

---

### 4. File / WebView fallback

For WKWebView, old Safari, or any environment where `getUserMedia` is unavailable. DocuSnap renders a built-in file-picker UI into `fallbackContainer`.

```html
<div id="fallbackContainer"></div>
```

```js
// Detect capability first; you can also let DocuSnap handle it automatically
// by passing null, null to start() — the result is the same.
const mode = DocuSnap.detectCapability(); // 'auto' or 'file'

const snap = new DocuSnap({
  captureMode: 'file',                                    // or omit and rely on 'smart'
  fallbackContainer: document.getElementById('fallbackContainer'),

  onFrame(frame) {
    // In file mode, instructionCode reflects post-upload quality:
    // CONFIRMED = quality passed, SEARCHING = quality suboptimal
    document.getElementById('status').textContent = frame.hint;
  },

  onCapture(cr) {
    // cr.image is the selected/taken photo as a Blob
    // cr.documentImage may be null if corners could not be found
    const url = URL.createObjectURL(cr.image);
    document.getElementById('preview').src = url;
    snap.destroy();
  },

  onError(err) {
    alert('Capture failed: ' + err.message);
  },
});

// Pass null, null — no video element or canvas needed in file mode
await snap.start(null, null);
```

If `fallbackContainer` is omitted in file mode, DocuSnap emits a single `onFrame` event with `hint: 'Tap to take a photo of the document'` and then waits. In that case you are responsible for opening a `<input type="file" accept="image/*" capture="environment">` and feeding the result back through your own pipeline, or for providing a container element.
