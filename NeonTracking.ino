/*
  ============================================================
  PART 1C — Green Marker Proximity-Based Queen Excluder Control
  ============================================================
  Save this as a tab named "NeonTracking.ino" in the same
  sketch folder as Part 1A. The Arduino IDE compiles all .ino
  tabs together, so servo1/servo2/SERVO1_OPEN_ANGLE/
  SERVO1_CLOSED_ANGLE/SERVO2_OPEN_ANGLE/SERVO2_CLOSED_ANGLE/
  servosCurrentlyOpen (defined in Part 1A) are directly usable
  here.

  Integration hooks required in Part 1A (already present if
  you're using the sketch from earlier in this conversation):

    1) Inside setupWebServer(), add:
         server.on("/stream", HTTP_GET, handleStream);

    2) Inside loop(), add:
         trackNeonColor();

  This version detects GREEN ONLY (fluorescent orange removed,
  since green is much easier to source/spot in practice).

  How "near" vs "far" is measured:
    The camera can't measure real-world distance directly, so
    this uses the SIZE of the detected green blob (number of
    matching pixels) as a proxy: a marker close to the lens
    fills more pixels, a marker far away fills fewer. You WILL
    need to calibrate GREEN_CLOSE_PIXEL_THRESHOLD and
    GREEN_FAR_PIXEL_THRESHOLD for your actual box/camera
    distance (see calibration notes below).

  Logic:
    - Green blob is LARGE (bee close to camera) -> CLOSE servos
    - Green blob is SMALL or absent (bee far/gone) -> OPEN servos
    - A dead zone between the two thresholds prevents rapid
      open/close flickering (hysteresis)
  ============================================================
*/

#include "img_converters.h"

// ============================================================
// Color Detection Threshold (RGB888) - GREEN ONLY
// ============================================================
// Fluorescent/bright green: high Green, low Red, low-medium Blue
#define GREEN_R_MAX   90
#define GREEN_G_MIN   170
#define GREEN_B_MAX   110

// Minimum matching pixels required to consider green "detected"
// at all (filters out noise/false positives)
#define MIN_GREEN_PIXELS   40

// Skip pixels during scan for performance (process every Nth pixel)
#define SCAN_STEP          2

// ============================================================
// Proximity Thresholds (CALIBRATE THESE FOR YOUR BOX)
// ============================================================
// Matched pixel count above which the bee is considered "close"
// enough to the camera to CLOSE the excluder gate.
#define GREEN_CLOSE_PIXEL_THRESHOLD   300

// Matched pixel count below which the bee is considered "far"
// enough to (eventually) OPEN the excluder gate.
// Lowered from 120 to 60 so a shrinking-but-still-present blob
// (motion blur, queen turning at an angle, partial occlusion)
// isn't mistaken for true absence and doesn't reopen the gate
// while the marker is technically still in frame.
#define GREEN_FAR_PIXEL_THRESHOLD     60

// Number of CONSECUTIVE "far/absent" scans required before the
// gate actually opens. Since trackNeonColor() runs every
// TRACK_INTERVAL (500ms), a value of 6 means green must be
// absent for roughly 3 continuous seconds before opening.
// This prevents a single noisy/missed frame from flicking the
// gate open right after the marker disappears.
#define GREEN_ABSENT_CONFIRM_CYCLES   6

// ------------------------------------------------------------
// CALIBRATION NOTES:
//   1. Place the green-marked queen at the distance from the
//      camera where you want the gate to just start closing.
//      Watch the Serial Monitor for "Matched Pixels: N" and
//      set GREEN_CLOSE_PIXEL_THRESHOLD near that N.
//   2. Move her to the distance where the gate should be fully
//      open, note that N, and set GREEN_FAR_PIXEL_THRESHOLD there.
//   3. Re-upload and verify behavior in real conditions
//      (natural light inside the box can shift readings).
// ------------------------------------------------------------

// ============================================================
// Tracking State Variables
// ============================================================
int  greenX = -1;              // Last detected X position (-1 = not found)
int  greenY = -1;              // Last detected Y position (-1 = not found)
bool greenDetected = false;    // Whether green was found in last scan
long lastMatchedPixels = 0;    // Size of the detected blob (proximity proxy)

// Timing for periodic tracking (runs independently of DHT/servo timing)
unsigned long lastTrackTime = 0;
const unsigned long TRACK_INTERVAL = 500; // Run detection every 500 ms

// Counts consecutive scans where green was absent/far, used to
// require sustained absence before opening (debounce)
int greenAbsentCycleCount = 0;

// ============================================================
// setupColorTracking()
// Initializes green marker proximity tracking (logs status;
// thresholds above are compile-time constants and can be tuned).
// ============================================================
void setupColorTracking() {
  Serial.println("Green marker proximity tracking initialized.");
}

// ============================================================
// isGreenMarker()
// Simple RGB threshold classifier for the green marker
// ============================================================
inline bool isGreenMarker(uint8_t r, uint8_t g, uint8_t b) {
  return (g >= GREEN_G_MIN) && (r <= GREEN_R_MAX) && (b <= GREEN_B_MAX);
}

// ============================================================
// updateServoByProximity()
// Evaluates the PROXIMITY half of the servo decision, based on
// the size of the detected green blob (proxy for how close the
// bee is to the camera). Uses hysteresis (two thresholds) to
// prevent flicker, then hands off to applyServoDecision()
// (defined in Part 1A) so temperature and proximity combine
// into one final servo state.
// ============================================================
void updateServoByProximity(long matchedPixels) {
  if (matchedPixels >= GREEN_CLOSE_PIXEL_THRESHOLD) {
    // Bee is close to the camera -> CLOSE immediately (safety-first,
    // no delay needed when the marker is clearly detected)
    proximityWantsClose = true;
    greenAbsentCycleCount = 0; // Reset: marker is present again
  } else if (matchedPixels <= GREEN_FAR_PIXEL_THRESHOLD) {
    // Bee appears far/absent this scan -> count it, but don't open
    // until it's been absent for GREEN_ABSENT_CONFIRM_CYCLES in a row
    greenAbsentCycleCount++;
    if (greenAbsentCycleCount >= GREEN_ABSENT_CONFIRM_CYCLES) {
      proximityWantsClose = false;
    }
  }
  // else: value falls in the dead zone between thresholds ->
  // deliberately keep current state and counter unchanged
  // (hysteresis, avoids flicker right at the boundary)

  applyServoDecision(); // Combine with temperature signal, act on both servos
}

// ============================================================
// trackNeonColor()
// Captures a frame, converts to RGB888, scans for the green
// marker, computes centroid X/Y and blob size, prints results,
// and drives the excluder servos by proximity.
// Call this from loop().
// ============================================================
void trackNeonColor() {
  unsigned long now = millis();
  if (now - lastTrackTime < TRACK_INTERVAL) {
    return; // Not time yet, keep loop responsive for streaming/web
  }
  lastTrackTime = now;

  // Grab a frame from the existing camera driver (JPEG format)
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Green Tracking: camera capture failed.");
    return;
  }

  // Allocate an RGB888 buffer for pixel analysis.
  // Use PSRAM specifically (heap_caps_malloc) instead of plain malloc().
  // A VGA/SVGA frame needs 300KB-1.4MB for this buffer, which is too
  // large for the ESP32's small internal RAM (~300KB free at runtime).
  size_t rgbLen = (size_t)fb->width * (size_t)fb->height * 3;
  uint8_t *rgbBuf = (uint8_t *)heap_caps_malloc(rgbLen, MALLOC_CAP_SPIRAM);

  if (!rgbBuf) {
    // Fallback: try internal RAM in case PSRAM isn't available/enabled
    rgbBuf = (uint8_t *)malloc(rgbLen);
  }

  if (!rgbBuf) {
    Serial.println("Green Tracking: RGB buffer allocation failed (check Tools > PSRAM is Enabled).");
    esp_camera_fb_return(fb);
    return;
  }

  bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, rgbBuf);

  if (!converted) {
    Serial.println("Green Tracking: JPEG->RGB888 conversion failed.");
    free(rgbBuf);
    esp_camera_fb_return(fb);
    return;
  }

  // Scan pixels, accumulate centroid for green marker
  long sumX = 0, sumY = 0, count = 0;

  int width  = fb->width;
  int height = fb->height;

  for (int y = 0; y < height; y += SCAN_STEP) {
    for (int x = 0; x < width; x += SCAN_STEP) {
      size_t idx = ((size_t)y * width + x) * 3;
      uint8_t r = rgbBuf[idx + 0];
      uint8_t g = rgbBuf[idx + 1];
      uint8_t b = rgbBuf[idx + 2];

      if (isGreenMarker(r, g, b)) {
        sumX += x;
        sumY += y;
        count++;
      }
    }
  }

  free(rgbBuf);
  esp_camera_fb_return(fb);

  // Decide if green was detected
  if (count >= MIN_GREEN_PIXELS) {
    greenX = sumX / count;
    greenY = sumY / count;
    greenDetected = true;
    lastMatchedPixels = count;
  } else {
    greenDetected = false;
    greenX = -1;
    greenY = -1;
    lastMatchedPixels = 0; // No marker seen -> treated as "far"
  }

  // Print result to Serial Monitor
  if (greenDetected) {
    Serial.print("Green Marker Detected | X: ");
    Serial.print(greenX);
    Serial.print(" | Y: ");
    Serial.print(greenY);
    Serial.print(" | Matched Pixels: ");
    Serial.println(lastMatchedPixels);
  } else {
    Serial.println("Green Tracking: no green marker detected (treated as FAR).");
  }

  // Drive the excluder servos based on blob size (proximity proxy)
  updateServoByProximity(lastMatchedPixels);
}

// ============================================================
// handleStream()
// MJPEG streaming handler compatible with the standard
// WebServer library (chunked transfer). Keeps live camera
// streaming available alongside dashboard and tracking.
// Access via: http://<device-ip>/stream
// ============================================================
void handleStream() {
  WiFiClient client = server.client();

  String response = "HTTP/1.1 200 OK\r\n";
  response += "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
  server.sendContent(response);

  while (client.connected()) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Stream: camera capture failed.");
      break;
    }

    String header = "--frame\r\n";
    header += "Content-Type: image/jpeg\r\n";
    header += "Content-Length: " + String(fb->len) + "\r\n\r\n";

    server.sendContent(header);
    client.write(fb->buf, fb->len);
    server.sendContent("\r\n");

    esp_camera_fb_return(fb);

    // Yield briefly so DHT reads / servo updates / tracking still run
    delay(30);

    if (!client.connected()) {
      break;
    }
  }
}
