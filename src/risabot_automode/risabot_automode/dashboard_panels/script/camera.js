function toggleCam() {
  const b = document.getElementById('camBtn');
  const d = document.getElementById('camContainer');
  const t = document.getElementById('camTabs');
  const s = document.getElementById('camSrcTabs');
  const off = document.getElementById('camOff');
  const img = document.getElementById('camImg');
  if(b.classList.contains('active')) {
    b.classList.remove('active');
    b.textContent = String.fromCodePoint(0x1F4F7) + ' Enable Camera';
    d.classList.remove('active');
    t.style.display = 'none';
    s.style.display = 'none';
    off.style.display = 'flex';
    stopCamStream();
    img.style.display = 'none';
    img.src = '';
  } else {
    b.classList.add('active');
    b.textContent = String.fromCodePoint(0x1F4F7) + ' Disable Camera';
    d.classList.add('active');
    t.style.display = 'flex';
    s.style.display = 'flex';
    off.style.display = 'none';
    img.style.display = 'block';
    // Trigger auto_toggle_debug on initial enable (default view is 'raw')
    fetch('/api/set_cam_view?view=raw&source=' + encodeURIComponent(camSource));
    startCamStream();
  }
}

// Robust MJPEG player: pulls /camera_feed with fetch and reassembles frames
// manually, so a dead connection on a flaky link is detected (no-bytes
// timeout) and the stream reconnects itself with backoff. Replaces naive
// <img src> streaming, which freezes forever on the last frame.
window._camPlayer = null;
function stopCamStream() {
  const st = window._camPlayer;
  window._camPlayer = null;
  if (st) {
    st.stopped = true;
    try { st.controller.abort(); } catch (e) {}
  }
}
function startCamStream() {
  const img = document.getElementById('camImg');
  if (!img) return;
  stopCamStream();
  const st = { stopped: false, controller: null, failures: 0 };
  window._camPlayer = st;
  const BND = [45, 45, 102, 114, 97, 109, 101]; // '--frame'
  const DH = [13, 10, 13, 10];                   // CR LF CR LF
  const LF = [10];                               // LF
  function findSeq(hay, needle, from) {
    for (let i = from; i <= hay.length - needle.length; i++) {
      let ok = true;
      for (let j = 0; j < needle.length; j++) {
        if (hay[i + j] !== needle[j]) { ok = false; break; }
      }
      if (ok) return i;
    }
    return -1;
  }
  function concat(a, b) {
    const c = new Uint8Array(a.length + b.length);
    c.set(a, 0); c.set(b, a.length);
    return c;
  }
  (async () => {
    while (!st.stopped) {
      const ctrl = new AbortController();
      st.controller = ctrl;
      try {
        const resp = await fetch('/camera_feed?' + Date.now(), { signal: ctrl.signal, cache: 'no-store' });
        if (!resp.ok || !resp.body) throw new Error('stream unavailable');
        const reader = resp.body.getReader();
        let buf = new Uint8Array(0);
        let needBoundary = true;
        let lastFrame = Date.now();
        st.failures = 0;
        const stallTimer = setInterval(() => {
          if (st.stopped) { clearInterval(stallTimer); return; }
          if (Date.now() - lastFrame > 5000) {
            clearInterval(stallTimer);
            try { ctrl.abort(); } catch (e) {}
          }
        }, 1000);
        try {
          for (;;) {
            if (st.stopped) break;
            const rd = await reader.read();
            if (rd.done) break;
            buf = concat(buf, rd.value);
            if (buf.length > 1048576) { buf = buf.slice(buf.length - 1048576); needBoundary = true; }
            let batchJpg = null;
            for (;;) {
              if (needBoundary) {
                const bi = findSeq(buf, BND, 0);
                if (bi < 0) { if (buf.length > 32) buf = buf.slice(buf.length - 32); break; }
                const le = findSeq(buf, LF, bi);
                if (le < 0) break;
                buf = buf.slice(le + 1);
                needBoundary = false;
              }
              const he = findSeq(buf, DH, 0);
              if (he < 0) break;
              let hstr = '';
              try { hstr = new TextDecoder().decode(buf.slice(0, he)); } catch (e) { hstr = ''; }
              let n = -1;
              const clKey = 'Content-Length:';
              const ci = hstr.indexOf(clKey);
              if (ci >= 0) {
                const dm = /^[0-9]+/.exec(hstr.slice(ci + clKey.length).trim());
                if (dm) n = parseInt(dm[0], 10);
              }
              if (n < 0) { buf = buf.slice(he + 4); needBoundary = true; continue; }
              const fs = he + 4;
              if (buf.length < fs + n) break;
              const jpg = buf.slice(fs, fs + n);
              buf = buf.slice(fs + n);
              needBoundary = true;
              batchJpg = jpg; // render only the newest below: drops stale backlog
              lastFrame = Date.now();
            }
            if (batchJpg) {
              const url = URL.createObjectURL(new Blob([batchJpg], { type: 'image/jpeg' }));
              const old = img.dataset ? img.dataset.blobUrl : null;
              img.src = url;
              if (img.dataset) img.dataset.blobUrl = url;
              if (old) { try { URL.revokeObjectURL(old); } catch (e) {} }
            }
          }
        } finally {
          clearInterval(stallTimer);
          try { await reader.cancel(); } catch (e) {}
        }
      } catch (e) {
        // dropped connection or abort: reconnect below unless stopped
      }
      if (st.stopped) break;
      st.failures += 1;
      await new Promise((r) => setTimeout(r, Math.min(800 + st.failures * 400, 4000)));
    }
  })();
}

let camSource = 'forward';
function setCamSource(source, btn) {
  document.querySelectorAll('#camSrcTabs .cam-tab').forEach(t => t.classList.remove('active'));
  btn.classList.add('active');
  camSource = source;
  // Side sources are raw-only: reset the view tabs to Raw.
  document.querySelectorAll('#camTabs .cam-tab').forEach(t => t.classList.remove('active'));
  document.querySelector('#camTabs .cam-tab').classList.add('active');
  fetch('/api/set_cam_view?view=raw&source=' + encodeURIComponent(source)).then(() => {
    const img = document.getElementById('camImg');
    if (img && img.style.display !== 'none') {
      startCamStream();
    }
  });
}

function setCamView(view, btn) {
  document.querySelectorAll('#camTabs .cam-tab').forEach(t => t.classList.remove('active'));
  btn.classList.add('active');
  fetch('/api/set_cam_view?view=' + encodeURIComponent(view) + '&source=' + encodeURIComponent(camSource)).then(() => {
    // Restart the robust stream reader to pick up the new view immediately
    const img = document.getElementById('camImg');
    if (img && img.style.display !== 'none') {
      startCamStream();
    }
  });
}

