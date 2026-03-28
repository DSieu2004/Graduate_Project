"""
Raspberry Pi — QR Scanner + MJPEG Streamer
===========================================
Runs on the Pi.  Does three things at once:

  1. Captures webcam frames continuously
  2. Decodes QR codes → POSTs to the Warehouse server  POST /scan
  3. Serves an MJPEG stream at /video for the dashboard Camera tab

Install on the Pi:
  pip install flask opencv-python pyzbar requests

Run:
  python pi_qr_scanner.py

Configure the four constants below (or override with env vars).
"""

import os
import cv2
import time
import threading
import requests
from flask import Flask, Response
from pyzbar.pyzbar import decode

# ─── Configuration ────────────────────────────────────────────────────────────
ROBOT_CODE = os.getenv("ROBOT_CODE", "AMR_01")
SERVER_URL = os.getenv("SERVER_URL", "http://192.168.1.57:8000")  # ← your PC IP
STREAM_PORT = int(os.getenv("STREAM_PORT", "5000"))
CAMERA_INDEX = int(os.getenv("CAMERA_INDEX", "0"))
COOLDOWN = float(os.getenv("COOLDOWN", "3.0"))  # seconds before same QR re-triggers
# ──────────────────────────────────────────────────────────────────────────────

app = Flask(__name__)

# Shared state — protected by lock
_lock = threading.Lock()
_frame = None  # latest JPEG-encoded bytes (annotated)

# Anti-spam memory
_last_qr = ""
_last_time = 0.0


def camera_loop():
    """Background thread: capture → QR detect → annotate → store frame."""
    global _frame, _last_qr, _last_time

    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print("[ERROR] Cannot open camera index", CAMERA_INDEX)
        return

    print(f"[CAM] Camera opened (index {CAMERA_INDEX})")

    while True:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.05)
            continue

        # ── QR detection ──────────────────────────────────────
        qr_codes = decode(frame)

        for qr in qr_codes:
            qr_data = qr.data.decode("utf-8", errors="replace")
            (x, y, w, h) = qr.rect

            # Draw green box + label on the frame
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
            cv2.putText(
                frame,
                qr_data,
                (x, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
            )

            # ── Anti-spam check ───────────────────────────────
            now = time.time()
            if qr_data != _last_qr or (now - _last_time) > COOLDOWN:
                _last_qr = qr_data
                _last_time = now

                # Draw "SCANNED ✓" flash
                cv2.putText(
                    frame,
                    "SCANNED",
                    (x, y + h + 25),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 0),
                    2,
                )

                # POST to server (fire-and-forget in thread to avoid blocking)
                threading.Thread(
                    target=_post_scan,
                    args=(qr_data,),
                    daemon=True,
                ).start()

        # ── Encode frame to JPEG for MJPEG stream ────────────
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            with _lock:
                _frame = buf.tobytes()


def _post_scan(qr_data: str):
    """Send the scan to the Warehouse server."""
    url = SERVER_URL.rstrip("/") + "/scan"
    payload = {"robot_code": ROBOT_CODE, "qr_code": qr_data}
    try:
        r = requests.post(url, json=payload, timeout=5)
        if r.ok:
            print(f"[SCAN] Reported '{qr_data}' → server OK")
        else:
            print(f"[SCAN] Server returned {r.status_code}: {r.text}")
    except requests.RequestException as e:
        print(f"[SCAN] Failed to reach server: {e}")


# ─── MJPEG streaming endpoint ────────────────────────────────────────────────


def _generate():
    """Yield MJPEG multipart frames."""
    while True:
        with _lock:
            jpg = _frame
        if jpg is None:
            time.sleep(0.05)
            continue
        yield (b"--frame\r\n" b"Content-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n")


@app.route("/video")
def video():
    return Response(
        _generate(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@app.route("/health")
def health():
    return {"robot": ROBOT_CODE, "camera": _frame is not None}


# ─── Main ─────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    print("=" * 56)
    print("  Warehouse Pi — QR Scanner + MJPEG Streamer")
    print(f"  Robot     : {ROBOT_CODE}")
    print(f"  Server    : {SERVER_URL}")
    print(f"  Stream    : http://0.0.0.0:{STREAM_PORT}/video")
    print(f"  Camera    : index {CAMERA_INDEX}")
    print(f"  Cooldown  : {COOLDOWN}s")
    print("=" * 56)

    # Start camera thread
    cam_thread = threading.Thread(target=camera_loop, daemon=True)
    cam_thread.start()

    # Start Flask (MJPEG server)
    app.run(host="0.0.0.0", port=STREAM_PORT, threaded=True)